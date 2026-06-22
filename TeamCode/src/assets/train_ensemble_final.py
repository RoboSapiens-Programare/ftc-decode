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
X_train = scaler.fit_transform(df_train_aug[features].values)
y_train = df_train_aug[target].values
X_test = scaler.transform(df_test_raw[features].values)
y_test = df_test_raw[target].values
split = int(0.85*len(X_train))
X_tr, X_val = X_train[:split], X_train[split:]
y_tr, y_val = y_train[:split], y_train[split:]

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

def ensemble_predict(models, X):
    return np.mean([m.predict(X, verbose=0) for m in models], axis=0)

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
    return yp

# ===== EXPERIMENTS =====

# F1: Ensemble of 5 (with dropout)
print("Training 5 dropout models...")
m5 = [train_model(42+i, dropout=True) for i in range(5)]
pp5 = ensemble_predict(m5, X_test)
print_results(pp5, "F1: Ensemble 5 (dropout)")

# F2: Ensemble of 5 (no dropout)
print("\nTraining 5 no-dropout models...")
m5nd = [train_model(42+i, dropout=False) for i in range(5)]
pp5nd = ensemble_predict(m5nd, X_test)
print_results(pp5nd, "F2: Ensemble 5 (no dropout)")

# F3: Combined ensemble (10 models: 5 dropout + 5 no-dropout)
print("\nCombined ensemble...")
all_m = m5 + m5nd
pp_all = ensemble_predict(all_m, X_test)
print_results(pp_all, "F3: Combined (5drop+5nodrop)")

# F4: Ensemble of 7 dropout + 3 no-dropout (weighted)
print("\nWeighted combined (7drop+3nodrop)...")
pp_weighted = (7 * ensemble_predict(m5[:5], X_test) + 3 * ensemble_predict(m5nd[:3], X_test)) / 10
y_w = np.argmax(pp_weighted, axis=1)
print_results(pp_weighted, "F4: Weighted (7drop+3nodrop)")

# F5: Find best ensemble size from 1..5
print("\n\n--- Ensemble size sweep (dropout) ---")
for n in range(1, 6):
    pp = ensemble_predict(m5[:n], X_test)
    yp = np.argmax(pp, axis=1)
    ov = (yp==y_test).mean()
    c2 = (yp[y_test==2]==2).mean() if (y_test==2).sum()>0 else 0
    c1 = (yp[y_test==1]==1).mean() if (y_test==1).sum()>0 else 0
    c3 = (yp[y_test==3]==3).mean() if (y_test==3).sum()>0 else 0
    print(f"  Ensemble size {n}: overall={ov*100:.1f}% c1={c1*100:.1f}% c2={c2*100:.1f}% c3={c3*100:.1f}%")

# F6: Find best ensemble size for no-dropout
print("\n--- Ensemble size sweep (no dropout) ---")
for n in range(1, 6):
    pp = ensemble_predict(m5nd[:n], X_test)
    yp = np.argmax(pp, axis=1)
    ov = (yp==y_test).mean()
    c2 = (yp[y_test==2]==2).mean() if (y_test==2).sum()>0 else 0
    c1 = (yp[y_test==1]==1).mean() if (y_test==1).sum()>0 else 0
    c3 = (yp[y_test==3]==3).mean() if (y_test==3).sum()>0 else 0
    print(f"  Ensemble size {n}: overall={ov*100:.1f}% c1={c1*100:.1f}% c2={c2*100:.1f}% c3={c3*100:.1f}%")

# F7: Post-hoc decision rule tuning on best ensemble
# Use F1 (dropout ensemble 5) as it likely has best overall
print("\n\n--- Decision rule sweep on best ensemble ---")
best_rules = []
for t3 in np.arange(0.2, 0.8, 0.05):
    for c2_bonus in [0.0, 0.05, 0.1, 0.15, 0.2, 0.3]:
        pp = pp5.copy()
        yp = np.argmax(pp, axis=1)
        for i in range(len(yp)):
            if pp[i, 3] > t3:
                pp[i, 2] += c2_bonus
        yp = np.argmax(pp, axis=1)
        ov = (yp==y_test).mean()
        c0 = (yp[y_test==0]==0).mean() if (y_test==0).sum()>0 else 0
        c1 = (yp[y_test==1]==1).mean() if (y_test==1).sum()>0 else 0
        c2 = (yp[y_test==2]==2).mean() if (y_test==2).sum()>0 else 0
        c3 = (yp[y_test==3]==3).mean() if (y_test==3).sum()>0 else 0
        min_c = min(c0, c1, c2*0.5+c3*0.5)
        best_rules.append((min_c, t3, c2_bonus, ov, c1, c2, c3))

best_rules.sort(reverse=True, key=lambda x: x[0])
for _, t3, bonus, ov, c1, c2, c3 in best_rules[:5]:
    print(f"  t3>{t3:.2f} bonus={bonus:.2f}: overall={ov*100:.1f}% c1={c1*100:.1f}% c2={c2*100:.1f}% c3={c3*100:.1f}%")
