import numpy as np, tensorflow as tf, json, os

np.random.seed(42); tf.random.set_seed(42)

X_tr = np.load("X_tr.npy"); y_tr = np.load("y_tr.npy").astype(np.int32)
X_val = np.load("X_val.npy"); y_val = np.load("y_val.npy").astype(np.int32)
X_test = np.load("X_test.npy"); y_test = np.load("y_test.npy").astype(np.int32)
teacher_tr = np.load("teacher_tr.npy")
teacher_val = np.load("teacher_val.npy")
teacher_test = np.load("teacher_test.npy")

print(f"Train: {len(X_tr)}, Val: {len(X_val)}, Test: {len(X_test)}")

with open("scaler_data.json") as f:
    scaler_data = json.load(f)
features = scaler_data["features"]

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

TEMP = 2.0
BATCH = 32
n_batches = int(np.ceil(len(X_tr) / BATCH))

best_overall = 0
best_c2 = 0
best_student_w = None
best_params = None

print("\nExperiment: Distillation with varying alpha")
for ALPHA in [0.0, 0.3, 0.5, 0.7]:
    student = make_student()
    opt = tf.keras.optimizers.Adam(0.001)
    best_val_loss = float('inf')
    best_w = None
    
    for epoch in range(60):
        idx = np.random.permutation(len(X_tr))
        Xs = X_tr[idx]; ys = y_tr[idx]; ts = teacher_tr[idx]
        total_loss = 0
        
        for b in range(n_batches):
            s_i, e_i = b*BATCH, min((b+1)*BATCH, len(X_tr))
            Xb = Xs[s_i:e_i]; yb = ys[s_i:e_i]; tb = ts[s_i:e_i]
            
            with tf.GradientTape() as tape:
                pp = student(Xb, training=True)
                hl = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(yb, pp))
                
                eps = 1e-7
                sl = tf.math.log(tf.clip_by_value(pp, eps, 1.0)) / TEMP
                tl = tf.math.log(tf.clip_by_value(tb, eps, 1.0)) / TEMP
                ss = tf.nn.softmax(sl); t_soft = tf.nn.softmax(tl)
                soft = tf.reduce_mean(tf.keras.losses.KLDivergence()(t_soft, ss)) * (TEMP ** 2)
                
                loss = (1 - ALPHA) * hl + ALPHA * soft
            
            grads = tape.gradient(loss, student.trainable_variables)
            opt.apply_gradients(zip(grads, student.trainable_variables))
            total_loss += loss.numpy()
        
        val_pp = student.predict(X_val, verbose=0)
        val_hard = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(y_val, val_pp)).numpy()
        
        if val_hard < best_val_loss:
            best_val_loss = val_hard
            best_w = student.get_weights()
    
    # Restore and evaluate
    if best_w is not None:
        student.set_weights(best_w)
    test_pp = student.predict(X_test, verbose=0)
    yp = np.argmax(test_pp, axis=1)
    overall = (yp == y_test).mean()
    c0 = (yp[y_test==0]==0).mean() if (y_test==0).sum()>0 else 0
    c1 = (yp[y_test==1]==1).mean() if (y_test==1).sum()>0 else 0
    c2 = (yp[y_test==2]==2).mean() if (y_test==2).sum()>0 else 0
    c3 = (yp[y_test==3]==3).mean() if (y_test==3).sum()>0 else 0
    print(f"  alpha={ALPHA:.1f}: overall={overall*100:.1f}% c0={c0*100:.1f}% c1={c1*100:.1f}% c2={c2*100:.1f}% c3={c3*100:.1f}%")
    
    if overall > best_overall or (overall == best_overall and c2 > best_c2):
        best_overall = overall
        best_student_w = best_w
        best_params = (ALPHA, c0, c1, c2, c3)

ALPHA, c0, c1, c2, c3 = best_params
print(f"\nBest: alpha={ALPHA} (overall={best_overall*100:.1f}%, c1={c1*100:.1f}%, c2={c2*100:.1f}%)")

# Retrain best and export
print("\nRetraining best student for export...")
student = make_student()
opt = tf.keras.optimizers.Adam(0.001)
best_val_loss = float('inf')
best_w = None

for epoch in range(80):
    idx = np.random.permutation(len(X_tr))
    Xs = X_tr[idx]; ys = y_tr[idx]; ts = teacher_tr[idx]
    
    for b in range(n_batches):
        s_i, e_i = b*BATCH, min((b+1)*BATCH, len(X_tr))
        Xb = Xs[s_i:e_i]; yb = ys[s_i:e_i]; tb = ts[s_i:e_i]
        
        with tf.GradientTape() as tape:
            pp = student(Xb, training=True)
            hl = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(yb, pp))
            eps = 1e-7
            sl = tf.math.log(tf.clip_by_value(pp, eps, 1.0)) / TEMP
            tl = tf.math.log(tf.clip_by_value(tb, eps, 1.0)) / TEMP
            ss = tf.nn.softmax(sl); t_soft = tf.nn.softmax(tl)
            soft = tf.reduce_mean(tf.keras.losses.KLDivergence()(t_soft, ss)) * (TEMP ** 2)
            loss = (1 - ALPHA) * hl + ALPHA * soft
        
        grads = tape.gradient(loss, student.trainable_variables)
        opt.apply_gradients(zip(grads, student.trainable_variables))
    
    val_pp = student.predict(X_val, verbose=0)
    val_hard = tf.reduce_mean(tf.keras.losses.sparse_categorical_crossentropy(y_val, val_pp)).numpy()
    if val_hard < best_val_loss:
        best_val_loss = val_hard
        best_w = student.get_weights()

if best_w is not None:
    student.set_weights(best_w)
test_pp = student.predict(X_test, verbose=0)
yp = np.argmax(test_pp, axis=1)

print(f"\n--- Final Distilled Student ---")
from sklearn.metrics import confusion_matrix
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum()>0: print(f"  Class {c}: {(yp[mask]==c).mean()*100:.1f}% ({mask.sum()} samples)")
print(f"  Overall: {(yp==y_test).mean()*100:.1f}%")
cm = confusion_matrix(y_test, yp)
header = "       " + "  ".join(f"P{c}" for c in range(4))
print(header)
for i in range(4): print(f"  T{i}:  " + " ".join(f"{cm[i,j]:4d}" for j in range(4)))

# Teacher comparison
print(f"\n--- Teacher Ensemble (for comparison) ---")
teacher_yp = np.argmax(teacher_test, axis=1)
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum()>0: print(f"  Class {c}: {(teacher_yp[mask]==c).mean()*100:.1f}%")
print(f"  Overall: {(teacher_yp==y_test).mean()*100:.1f}%")

# Export TFLite
print("\nExporting to TFLite...")
converter = tf.lite.TFLiteConverter.from_keras_model(student)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_types = [tf.float16]
tflite_model = converter.convert()
with open("shoot_predictor.tflite", "wb") as f:
    f.write(tflite_model)

# Verify
interpreter = tf.lite.Interpreter(model_content=tflite_model)
interpreter.allocate_tensors()
inp = interpreter.get_input_details()[0]
out = interpreter.get_output_details()[0]
print(f"Input shape: {inp['shape']}, dtype: {inp['dtype']}")
print(f"Output shape: {out['shape']}, dtype: {out['dtype']}")

# Save scaler
with open("scaler_values.json", "w") as f:
    json.dump(scaler_data, f, indent=2)
print("\nScaler values for Java code:")
for i, feat in enumerate(features):
    print(f"  {feat}: mean={scaler_data['means'][i]:.3f}, std={scaler_data['stds'][i]:.3f}")
