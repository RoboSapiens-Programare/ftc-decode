import os, random, json, numpy as np, pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
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

print("Step 1: Training teacher ensemble...")
from time import time
t0 = time()
teachers_drop = [train_model(42+i, dropout=True) for i in range(3)]
teachers_nodrop = [train_model(100+i, dropout=False) for i in range(3)]
all_teachers = teachers_drop + teachers_nodrop
t1 = time()
print(f"Teachers trained in {t1-t0:.1f}s")

def ensemble_pred(models, X):
    return np.mean([m.predict(X, verbose=0) for m in models], axis=0)

print("Step 2: Computing ensemble predictions...")
teacher_tr = ensemble_pred(all_teachers, X_tr)
teacher_val = ensemble_pred(all_teachers, X_val)
teacher_test = ensemble_pred(all_teachers, X_test)

np.save("teacher_tr.npy", teacher_tr)
np.save("teacher_val.npy", teacher_val)
np.save("teacher_test.npy", teacher_test)
np.save("X_tr.npy", X_tr); np.save("y_tr.npy", y_tr)
np.save("X_val.npy", X_val); np.save("y_val.npy", y_val)
np.save("X_test.npy", X_test); np.save("y_test.npy", y_test)
import json
with open("scaler_data.json", "w") as f:
    json.dump({"means": [float(scaler.mean_[i]) for i in range(len(features))],
               "stds": [float(np.sqrt(scaler.var_[i])) for i in range(len(features))],
               "features": features}, f, indent=2)

print("Done! Saved ensemble predictions and data.")
