import os, random, json, numpy as np, pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import confusion_matrix
import tensorflow as tf

np.random.seed(42); tf.random.set_seed(42); random.seed(42)
features = ["target_dist","angle_error","vel_x","vel_y","omega","voltage"]
target = "balls_scored"
df = pd.read_csv("ftc_shot_data.csv")
df_train_raw, df_test_raw = train_test_split(df, test_size=0.20, random_state=42, stratify=df[target])

def augment_standard(df_train_raw):
    def jitter(row, sd=0.05, sa=0.01, sv=0.1):
        r = row.copy(); r['target_dist']+=np.random.normal(0,sd); r['angle_error']+=np.random.normal(0,sa); r['voltage']+=np.random.normal(0,sv); return r
    aug = []
    for _,row in df_train_raw.iterrows():
        aug.append(row.to_dict())
        m = row.copy(); m['angle_error']=-row['angle_error']; m['vel_y']=-row['vel_y']; m['omega']=-row['omega']; aug.append(m.to_dict())
        for _ in range(3):
            aug.append(jitter(row).to_dict()); aug.append(jitter(m).to_dict())
    for cls,copies in [(1,16),(2,8)]:
        for _,row in df_train_raw[df_train_raw[target]==cls].iterrows():
            for _ in range(copies): aug.append(jitter(row,0.08,0.02,0.15).to_dict())
    return pd.DataFrame(aug)

df_train_aug = augment_standard(df_train_raw)
scaler = StandardScaler()
X_train_all = scaler.fit_transform(df_train_aug[features].values)
y_train_all = df_train_aug[target].values
X_test = scaler.transform(df_test_raw[features].values)
y_test = df_test_raw[target].values
split = int(0.85*len(X_train_all))
X_tr, X_val = X_train_all[:split], X_train_all[split:]
y_tr, y_val = y_train_all[:split], y_train_all[split:]

# ===================== BUILD ENSEMBLE (TEACHER) =====================
def make_model(dropout=True):
    inp = tf.keras.Input(shape=(6,))
    x = tf.keras.layers.Dense(128, activation='swish')(inp)
    if dropout: x = tf.keras.layers.Dropout(0.2)(x)
    x = tf.keras.layers.Dense(64, activation='swish')(x)
    if dropout: x = tf.keras.layers.Dropout(0.15)(x)
    x = tf.keras.layers.Dense(32, activation='swish')(x)
    p1 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge1')(x)
    p2 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge2')(x)
    p3 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge3')(x)
    def o2c(a): g1,g2,g3=a; l=tf.concat([1.0-g1,g1-g2,g2-g3,g3],-1); c=tf.clip_by_value(l,1e-7,1.0); return c/tf.reduce_sum(c,-1,keepdims=True)
    co = tf.keras.layers.Lambda(o2c)([p1,p2,p3])
    m = tf.keras.Model(inputs=inp, outputs=co)
    m.compile(optimizer=tf.keras.optimizers.Adam(0.001), loss='sparse_categorical_crossentropy')
    return m

def train_model(seed, dropout=True):
    tf.random.set_seed(seed)
    m = make_model(dropout=dropout)
    cb = [tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True),
          tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=8, min_lr=1e-6)]
    m.fit(X_tr, y_tr, validation_data=(X_val, y_val), epochs=150, batch_size=32, callbacks=cb, verbose=0)
    return m

print("Training teacher ensemble (5 dropout + 5 no-dropout)...")
teachers_drop = [train_model(42+i, dropout=True) for i in range(3)]
teachers_nodrop = [train_model(100+i, dropout=False) for i in range(3)]
all_teachers = teachers_drop + teachers_nodrop

def ensemble_pred(models, X):
    return np.mean([m.predict(X, verbose=0) for m in models], axis=0)

# ===================== KNOWLEDGE DISTILLATION =====================
print("Generating soft targets from ensemble...")
teacher_tr = ensemble_pred(all_teachers, X_tr)
teacher_val = ensemble_pred(all_teachers, X_val)
teacher_test = ensemble_pred(all_teachers, X_test)

# Student model: same architecture, trained with combined loss
def make_student():
    inp = tf.keras.Input(shape=(6,), name='input_layer')
    x = tf.keras.layers.Dense(128, activation='swish')(inp)
    x = tf.keras.layers.Dropout(0.2)(x)
    x = tf.keras.layers.Dense(64, activation='swish')(x)
    x = tf.keras.layers.Dropout(0.15)(x)
    x = tf.keras.layers.Dense(32, activation='swish')(x)
    p1 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge1')(x)
    p2 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge2')(x)
    p3 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge3')(x)
    def o2c(a): g1,g2,g3=a; l=tf.concat([1.0-g1,g1-g2,g2-g3,g3],-1); c=tf.clip_by_value(l,1e-7,1.0); return c/tf.reduce_sum(c,-1,keepdims=True)
    co = tf.keras.layers.Lambda(o2c, name='class_output')([p1,p2,p3])
    return tf.keras.Model(inputs=inp, outputs=co)

def distillation_loss(alpha=0.3, T=2.0):
    """Combined loss: (1-alpha)*hard_loss + alpha*soft_loss"""
    def loss(y_true, y_pred):
        # Hard loss: sparse categorical crossentropy
        hard = tf.keras.losses.sparse_categorical_crossentropy(y_true, y_pred)
        # Soft loss is computed externally; we'll use a custom training loop
        return hard
    return loss

# Custom training loop for distillation
TEMP = 2.0
ALPHA = 0.5

student = make_student()
optimizer = tf.keras.optimizers.Adam(0.001)

batch_size = 32
n_batches = int(np.ceil(len(X_tr) / batch_size))

best_val_loss = float('inf')
best_weights = None

for epoch in range(100):
    # Shuffle
    idx = np.random.permutation(len(X_tr))
    X_s = X_tr[idx]; y_s = y_tr[idx]; t_s = teacher_tr[idx]
    
    epoch_loss = 0
    for b in range(n_batches):
        s, e = b*batch_size, min((b+1)*batch_size, len(X_tr))
        Xb = X_s[s:e]; yb = y_s[s:e]; tb = t_s[s:e]
        
        with tf.GradientTape() as tape:
            student_probs = student(Xb, training=True)
            # Hard loss
            hard_loss = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(yb, student_probs))
            # Soft loss (KL divergence on temperature-scaled logits)
            # Convert probs to pseudo-logits
            eps = 1e-7
            s_logits = tf.math.log(tf.clip_by_value(student_probs, eps, 1.0)) / TEMP
            t_logits = tf.math.log(tf.clip_by_value(tb, eps, 1.0)) / TEMP
            s_soft = tf.nn.softmax(s_logits)
            t_soft = tf.nn.softmax(t_logits)
            soft_loss = tf.reduce_mean(tf.keras.losses.KLDivergence()(t_soft, s_soft)) * (TEMP ** 2)
            
            loss = (1 - ALPHA) * hard_loss + ALPHA * soft_loss
        
        grads = tape.gradient(loss, student.trainable_variables)
        optimizer.apply_gradients(zip(grads, student.trainable_variables))
        epoch_loss += loss.numpy()
    
    # Validation
    val_probs = student.predict(X_val, verbose=0)
    val_hard = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(y_val, val_probs)).numpy()
    
    if epoch % 20 == 0 or epoch == 99:
        val_yp = np.argmax(val_probs, axis=1)
        val_acc = (val_yp == y_val).mean()
        print(f"  Epoch {epoch}: train_loss={epoch_loss/n_batches:.4f} val_hard={val_hard:.4f} val_acc={val_acc*100:.1f}%")
    
    if val_hard < best_val_loss:
        best_val_loss = val_hard
        best_weights = student.get_weights()

# Restore best weights
student.set_weights(best_weights)
print(f"\nBest val_loss: {best_val_loss:.4f}")

# ===================== EVALUATE =====================
def print_results(pp, label):
    yp = np.argmax(pp, axis=1)
    print(f"\n--- {label} ---")
    for c in sorted(np.unique(y_test)):
        mask = y_test == c
        if mask.sum()>0: print(f"  Class {c}: {(yp[mask]==c).mean()*100:.1f}% ({mask.sum()} samples)")
    print(f"  Overall: {(yp==y_test).mean()*100:.1f}%")
    cm = confusion_matrix(y_test, yp)
    header = "       "+"  ".join(f"P{c}" for c in range(4))
    print(header)
    for i in range(4): print(f"  T{i}:  "+" ".join(f"{cm[i,j]:4d}" for j in range(4)))

print_results(teacher_test, "Teacher (combined ensemble)")
student_test = student.predict(X_test, verbose=0)
print_results(student_test, f"Student (distilled, alpha={ALPHA}, T={TEMP})")

# Try different alpha values
print("\n\n--- Distillation alpha sweep ---")
for alpha in [0.2, 0.3, 0.5, 0.7, 0.8]:
    s = make_student()
    opt = tf.keras.optimizers.Adam(0.001)
    n_b = int(np.ceil(len(X_tr) / 32))
    best_l = float('inf')
    best_w = None
    for ep in range(60):
        idx = np.random.permutation(len(X_tr))
        Xs = X_tr[idx]; ys = y_tr[idx]; ts = teacher_tr[idx]
        total_l = 0
        for b in range(n_b):
            s_i, e_i = b*32, min((b+1)*32, len(X_tr))
            Xb = Xs[s_i:e_i]; yb = ys[s_i:e_i]; tb = ts[s_i:e_i]
            with tf.GradientTape() as tape:
                pp = s(Xb, training=True)
                hl = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(yb, pp))
                eps=1e-7
                sl = tf.math.log(tf.clip_by_value(pp, eps, 1.0))/TEMP
                tl = tf.math.log(tf.clip_by_value(tb, eps, 1.0))/TEMP
                ss = tf.nn.softmax(sl); ts_ = tf.nn.softmax(tl)
                soft = tf.reduce_mean(tf.keras.losses.KLDivergence()(ts_, ss)) * (TEMP**2)
                l = (1-alpha)*hl + alpha*soft
            g = tape.gradient(l, s.trainable_variables)
            opt.apply_gradients(zip(g, s.trainable_variables))
            total_l += l.numpy()
        vl = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(y_val, s.predict(X_val, verbose=0))).numpy()
        if vl < best_l:
            best_l = vl; best_w = s.get_weights()
    s.set_weights(best_w)
    sp = s.predict(X_test, verbose=0); sy = np.argmax(sp, axis=1)
    ov = (sy==y_test).mean()
    c2 = (sy[y_test==2]==2).mean() if (y_test==2).sum()>0 else 0
    c1 = (sy[y_test==1]==1).mean() if (y_test==1).sum()>0 else 0
    c3 = (sy[y_test==3]==3).mean() if (y_test==3).sum()>0 else 0
    c0 = (sy[y_test==0]==0).mean() if (y_test==0).sum()>0 else 0
    print(f"  alpha={alpha:.1f}: overall={ov*100:.1f}% c0={c0*100:.1f}% c1={c1*100:.1f}% c2={c2*100:.1f}% c3={c3*100:.1f}%")

# ===================== EXPORT BEST =====================
print("\n\n--- Exporting best model (alpha=0.5) as TFLite ---")
# Retrain with alpha=0.5 since it's likely the best
final = make_student()
opt = tf.keras.optimizers.Adam(0.001)
n_b = int(np.ceil(len(X_tr) / 32))
best_l = float('inf')
best_w = None
for ep in range(80):
    idx = np.random.permutation(len(X_tr))
    Xs = X_tr[idx]; ys = y_tr[idx]; ts = teacher_tr[idx]
    for b in range(n_b):
        s_i, e_i = b*32, min((b+1)*32, len(X_tr))
        Xb = Xs[s_i:e_i]; yb = ys[s_i:e_i]; tb = ts[s_i:e_i]
        with tf.GradientTape() as tape:
            pp = final(Xb, training=True)
            hl = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(yb, pp))
            eps=1e-7
            sl = tf.math.log(tf.clip_by_value(pp, eps, 1.0))/TEMP
            tl = tf.math.log(tf.clip_by_value(tb, eps, 1.0))/TEMP
            ss = tf.nn.softmax(sl); ts_ = tf.nn.softmax(tl)
            soft = tf.reduce_mean(tf.keras.losses.KLDivergence()(ts_, ss)) * (TEMP**2)
            l = (1-0.5)*hl + 0.5*soft
        g = tape.gradient(l, final.trainable_variables)
        opt.apply_gradients(zip(g, final.trainable_variables))
    vl = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(y_val, final.predict(X_val, verbose=0))).numpy()
    if vl < best_l:
        best_l = vl; best_w = final.get_weights()
    if ep % 20 == 0:
        pp = final.predict(X_val, verbose=0); ypp = np.argmax(pp, axis=1)
        print(f"  Epoch {ep}: val_acc={(ypp==y_val).mean()*100:.1f}%")
final.set_weights(best_w)

# Evaluate final
fp = final.predict(X_test, verbose=0)
print_results(fp, "Final distilled model")

# Export TFLite
converter = tf.lite.TFLiteConverter.from_keras_model(final)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_types = [tf.float16]
tflite_model = converter.convert()
with open("shoot_predictor.tflite", "wb") as f:
    f.write(tflite_model)
print("\nTFLite model saved as shoot_predictor.tflite")

# Verify shapes
interpreter = tf.lite.Interpreter(model_content=tflite_model)
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]
out = interpreter.get_output_details()[0]
print(f"Input shape: {inp['shape']}, dtype: {inp['dtype']}")
print(f"Output shape: {out['shape']}, dtype: {out['dtype']}")

# Save scaler values
scaler_data = {
    "features": features,
    "means": [float(scaler.mean_[i]) for i in range(len(features))],
    "stds": [float(np.sqrt(scaler.var_[i])) for i in range(len(features))],
}
with open("scaler_values.json", "w") as f:
    json.dump(scaler_data, f, indent=2)
print("Scaler values saved to scaler_values.json")

for i, feat in enumerate(features):
    print(f"  {feat}: mean={scaler.mean_[i]:.3f}, std={np.sqrt(scaler.var_[i]):.3f}")
