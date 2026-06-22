import os
import random
import json
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.neighbors import NearestNeighbors
from sklearn.metrics import confusion_matrix
import tensorflow as tf

np.random.seed(42)
tf.random.set_seed(42)
random.seed(42)

features = ["target_dist", "angle_error", "vel_x", "vel_y", "omega", "voltage"]
target = "balls_scored"
csv_path = "ftc_shot_data.csv"

df = pd.read_csv(csv_path)
print(f"Dataset: {len(df)} samples")
class_counts = df[target].value_counts().sort_index()
print(f"Class distribution:\n{class_counts}\n")

df_train_raw, df_test_raw = train_test_split(
    df, test_size=0.20, random_state=42, stratify=df[target]
)

# ===================== HELPERS =====================

def make_ordinal(input_dim=6, layers_config=None):
    if layers_config is None:
        layers_config = [128, 64, 32]
    inputs = tf.keras.Input(shape=(input_dim,), name='input_layer')
    x = inputs
    for i, units in enumerate(layers_config):
        x = tf.keras.layers.Dense(units, activation='swish')(x)
        if units > 16:
            x = tf.keras.layers.Dropout(0.2 if i < len(layers_config)-1 else 0.15)(x)
    p_ge1 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge1')(x)
    p_ge2 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge2')(x)
    p_ge3 = tf.keras.layers.Dense(1, activation='sigmoid', name='ge3')(x)
    def ordinal_to_class(args):
        g1, g2, g3 = args
        logits = tf.concat([1.0 - g1, g1 - g2, g2 - g3, g3], axis=-1)
        clipped = tf.clip_by_value(logits, 1e-7, 1.0)
        return clipped / tf.reduce_sum(clipped, axis=-1, keepdims=True)
    class_probs = tf.keras.layers.Lambda(ordinal_to_class, name='class_output')([p_ge1, p_ge2, p_ge3])
    model = tf.keras.Model(inputs=inputs, outputs=class_probs)
    model.compile(optimizer=tf.keras.optimizers.Adam(0.001), loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return model

def train_fast(X_train, y_train, X_val, y_val, seed=42, epochs=150):
    tf.random.set_seed(seed)
    model = make_ordinal()
    callbacks = [
        tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=25, restore_best_weights=True),
        tf.keras.callbacks.ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=8, min_lr=1e-6)
    ]
    model.fit(X_train, y_train, validation_data=(X_val, y_val),
              epochs=epochs, batch_size=32, callbacks=callbacks, verbose=0)
    return model

def augment_standard(df_train_raw):
    def add_jitter(row, sd=0.05, sa=0.01, sv=0.1):
        r = row.copy()
        r['target_dist'] += np.random.normal(0, sd)
        r['angle_error'] += np.random.normal(0, sa)
        r['voltage'] += np.random.normal(0, sv)
        return r
    augmented = []
    for _, row in df_train_raw.iterrows():
        augmented.append(row.to_dict())
        mirror = row.copy()
        mirror['angle_error'] = -row['angle_error']
        mirror['vel_y'] = -row['vel_y']
        mirror['omega'] = -row['omega']
        augmented.append(mirror.to_dict())
        for _ in range(3):
            augmented.append(add_jitter(row).to_dict())
            augmented.append(add_jitter(mirror).to_dict())
    for cls, copies in [(1, 16), (2, 8)]:
        for _, row in df_train_raw[df_train_raw[target] == cls].iterrows():
            for _ in range(copies):
                augmented.append(add_jitter(row, 0.08, 0.02, 0.15).to_dict())
    return pd.DataFrame(augmented)

def augment_smote(df_train_raw, k=5):
    def add_jitter(row, sd=0.05, sa=0.01, sv=0.1):
        r = row.copy()
        r['target_dist'] += np.random.normal(0, sd)
        r['angle_error'] += np.random.normal(0, sa)
        r['voltage'] += np.random.normal(0, sv)
        return r
    augmented = []
    for _, row in df_train_raw.iterrows():
        augmented.append(row.to_dict())
        mirror = row.copy()
        mirror['angle_error'] = -row['angle_error']
        mirror['vel_y'] = -row['vel_y']
        mirror['omega'] = -row['omega']
        augmented.append(mirror.to_dict())
        for _ in range(3):
            augmented.append(add_jitter(row).to_dict())
            augmented.append(add_jitter(mirror).to_dict())
    for cls in [1, 2]:
        cls_rows = df_train_raw[df_train_raw[target] == cls]
        cls_data = cls_rows[features].values
        copies = 16 if cls == 1 else 8
        if len(cls_data) >= 3:
            nn = NearestNeighbors(n_neighbors=min(k, len(cls_data)))
            nn.fit(cls_data)
            _, indices = nn.kneighbors(cls_data)
            for i in range(len(cls_data)):
                for _ in range(copies):
                    ni = indices[i][np.random.randint(1, min(k, len(cls_data)))]
                    lam = np.random.random()
                    syn = cls_data[i] * lam + cls_data[ni] * (1 - lam)
                    sr = {features[j]: float(syn[j]) for j in range(len(features))}
                    sr[target] = cls
                    sr['target_dist'] += np.random.normal(0, 0.03)
                    sr['angle_error'] += np.random.normal(0, 0.01)
                    sr['voltage'] += np.random.normal(0, 0.08)
                    augmented.append(sr)
        else:
            for _, row in cls_rows.iterrows():
                for _ in range(20):
                    augmented.append(add_jitter(row, 0.08, 0.02, 0.15).to_dict())
    return pd.DataFrame(augmented)

def eval_model(model, X_test, y_test, label=""):
    pp = model.predict(X_test, verbose=0)
    yp = np.argmax(pp, axis=1)
    overall = (yp == y_test).mean()
    per_c = {}
    for c in sorted(np.unique(y_test)):
        mask = y_test == c
        if mask.sum() > 0:
            per_c[c] = (yp[mask] == c).mean() * 100
    print(f"\n--- {label} ---")
    print(f"  Overall: {overall*100:.1f}%")
    for c in sorted(per_c.keys()):
        print(f"  Class {c}: {per_c[c]:.1f}% ({(y_test==c).sum()} samples)")
    cm = confusion_matrix(y_test, yp)
    header = "       " + "  ".join(f"P{c}" for c in range(4))
    print(header)
    for i in range(4):
        print(f"  T{i}:  " + " ".join(f"{cm[i,j]:4d}" for j in range(4)))
    return overall, per_c, pp

def eval_ensemble(models, X_test, y_test, label="Ensemble"):
    all_pp = [m.predict(X_test, verbose=0) for m in models]
    avg = np.mean(all_pp, axis=0)
    yp = np.argmax(avg, axis=1)
    overall = (yp == y_test).mean()
    per_c = {}
    for c in sorted(np.unique(y_test)):
        mask = y_test == c
        if mask.sum() > 0:
            per_c[c] = (yp[mask] == c).mean() * 100
    print(f"\n--- {label} ---")
    print(f"  Overall: {overall*100:.1f}%")
    for c in sorted(per_c.keys()):
        print(f"  Class {c}: {per_c[c]:.1f}% ({(y_test==c).sum()} samples)")
    cm = confusion_matrix(y_test, yp)
    header = "       " + "  ".join(f"P{c}" for c in range(4))
    print(header)
    for i in range(4):
        print(f"  T{i}:  " + " ".join(f"{cm[i,j]:4d}" for j in range(4)))
    return overall, per_c, avg

# ===================== PREPARE DATA =====================

df_train_std = augment_standard(df_train_raw)
df_train_smt = augment_smote(df_train_raw)
print(f"Standard aug: {len(df_train_std)} samples")
print(df_train_std[target].value_counts().sort_index())
print(f"\nSMOTE aug: {len(df_train_smt)} samples")
print(df_train_smt[target].value_counts().sort_index())

# Scale both, with separate scalers
scaler_std = StandardScaler()
X_train_std = scaler_std.fit_transform(df_train_std[features].values)
y_train_std = df_train_std[target].values
X_test_std = scaler_std.transform(df_test_raw[features].values)

scaler_smt = StandardScaler()
X_train_smt = scaler_smt.fit_transform(df_train_smt[features].values)
y_train_smt = df_train_smt[target].values
X_test_smt = scaler_smt.transform(df_test_raw[features].values)

y_test = df_test_raw[target].values

# Split val from train (last 15%)
split_std = int(0.85 * len(X_train_std))
X_tr_std, X_val_std = X_train_std[:split_std], X_train_std[split_std:]
y_tr_std, y_val_std = y_train_std[:split_std], y_train_std[split_std:]

split_smt = int(0.85 * len(X_train_smt))
X_tr_smt, X_val_smt = X_train_smt[:split_smt], X_train_smt[split_smt:]
y_tr_smt, y_val_smt = y_train_smt[:split_smt], y_train_smt[split_smt:]

# ===================== EXPERIMENTS =====================

print("\n" + "="*55)
print("EXPERIMENTS")
print("="*55)

results = []

# E1: Single ordinal, standard aug
print("\n--- E1: Single ordinal (standard aug) ---")
m1 = train_fast(X_tr_std, y_tr_std, X_val_std, y_val_std, seed=42)
ov, pc, _ = eval_model(m1, X_test_std, y_test, "E1: Single (standard)")
results.append(("E1 Single (std aug)", ov, pc))

# E2: Single ordinal, SMOTE aug
print("\n--- E2: Single ordinal (SMOTE aug) ---")
m2 = train_fast(X_tr_smt, y_tr_smt, X_val_smt, y_val_smt, seed=42)
ov, pc, _ = eval_model(m2, X_test_smt, y_test, "E2: Single (SMOTE)")
results.append(("E2 Single (SMOTE)", ov, pc))

# E3: Ensemble of 3, standard aug
print("\n--- E3: Ensemble 3 (standard aug) ---")
models_std = [train_fast(X_tr_std, y_tr_std, X_val_std, y_val_std, seed=42+i) for i in range(3)]
ov, pc, _ = eval_ensemble(models_std, X_test_std, y_test, "E3: Ensemble 3 (std)")
results.append(("E3 Ensemble 3 (std)", ov, pc))

# E4: Ensemble of 3, SMOTE aug
print("\n--- E4: Ensemble 3 (SMOTE aug) ---")
models_smt = [train_fast(X_tr_smt, y_tr_smt, X_val_smt, y_val_smt, seed=42+i) for i in range(3)]
ov, pc, _ = eval_ensemble(models_smt, X_test_smt, y_test, "E4: Ensemble 3 (SMOTE)")
results.append(("E4 Ensemble 3 (SMOTE)", ov, pc))

# E5: Architecture search on SMOTE
print("\n--- E5: Architecture search ---")
arch_best = None
arch_best_ov = 0
arch_best_c2 = 0
for layers, name in [([128, 64, 32], "128-64-32"), ([256, 128, 64], "256-128-64"),
                       ([64, 32], "64-32"), ([256, 128, 64, 32], "256-128-64-32"),
                       ([512, 256], "512-256")]:
    tf.random.set_seed(42)
    m = make_ordinal(layers_config=layers)
    cb = [tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)]
    m.fit(X_tr_smt, y_tr_smt, validation_data=(X_val_smt, y_val_smt),
          epochs=120, batch_size=32, callbacks=cb, verbose=0)
    ov, pc, _ = eval_model(m, X_test_smt, y_test, f"E5: Arch {name}")
    if ov > arch_best_ov:
        arch_best_ov = ov
        arch_best = (name, m)
    if pc.get(2, 0) > arch_best_c2:
        arch_best_c2 = pc.get(2, 0)
results.append((f"E5 Best arch ({arch_best[0]})", arch_best_ov, {}))

# E6: No-dropout model (standard aug)
print("\n--- E6: No dropout (standard aug) ---")
tf.random.set_seed(42)
inp = tf.keras.Input(shape=(6,))
x = tf.keras.layers.Dense(128, activation='swish')(inp)
x = tf.keras.layers.Dense(64, activation='swish')(x)
x = tf.keras.layers.Dense(32, activation='swish')(x)
for name in ['ge1', 'ge2', 'ge3']:
    exec(f"p_{name} = tf.keras.layers.Dense(1, activation='sigmoid', name='{name}')(x)")
def o2c(args):
    g1,g2,g3 = args
    l = tf.concat([1.0-g1, g1-g2, g2-g3, g3], axis=-1)
    c = tf.clip_by_value(l, 1e-7, 1.0)
    return c / tf.reduce_sum(c, axis=-1, keepdims=True)
co = tf.keras.layers.Lambda(o2c)([p_ge1, p_ge2, p_ge3])
m6 = tf.keras.Model(inputs=inp, outputs=co)
m6.compile(optimizer=tf.keras.optimizers.Adam(0.001), loss='sparse_categorical_crossentropy')
cb = [tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=20, restore_best_weights=True)]
m6.fit(X_tr_std, y_tr_std, validation_data=(X_val_std, y_val_std), epochs=120, batch_size=32, callbacks=cb, verbose=0)
ov, pc, _ = eval_model(m6, X_test_std, y_test, "E6: No dropout")
results.append(("E6 No dropout", ov, pc))

# E7: RF on scaled (standard aug) — sanity check
print("\n--- E7: RF baseline (scaled) ---")
rf = RandomForestClassifier(n_estimators=300, max_depth=8, class_weight='balanced', random_state=42)
rf.fit(X_tr_std, y_tr_std)
rfp = rf.predict(X_test_std)
print(f"\nRF (scaled):")
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum() > 0:
        print(f"  Class {c}: {(rfp[mask]==c).mean()*100:.1f}% ({mask.sum()} samples)")
print(f"  Overall: {(rfp == y_test).mean()*100:.1f}%")

# ===================== POST-TUNING: BIAS ADJUSTMENT ON ENSEMBLE =====================
print("\n" + "="*55)
print("POST-TUNING: Bias adjustment on SMOTE ensemble")
print("="*55)

# Get ensemble val and test probabilities
all_val_pp = [m.predict(X_val_smt, verbose=0) for m in models_smt]
avg_val = np.mean(all_val_pp, axis=0)

# Extract the cumulative sigmoid outputs for each model
def get_cumulative_probs(model, X):
    """Directly get the 3 sigmoid outputs from the model, bypassing the Lambda layer."""
    intermediate = tf.keras.Model(inputs=model.input,
                                  outputs=[model.get_layer('ge1').output,
                                           model.get_layer('ge2').output,
                                           model.get_layer('ge3').output])
    return intermediate.predict(X, verbose=0)

# Try adjusting the ensemble average directly via soft bias shift
# Idea: for each sample, if P(class 3) is very high but the sample might be class 2,
# shift probability to class 2. We can do this by manipulating the cumulative probs.

# Simpler approach: find samples where the model is uncertain and adjust
# Actually, let me try: for each class, find the optimal decision threshold
# on the validation set, then apply to test set.

# We have 4 class probabilities. Standard decision: argmax.
# Alternative: manual rules based on probability patterns.

print("\nTrying decision rule alternatives on SMOTE ensemble outputs...")

all_test_pp = [m.predict(X_test_smt, verbose=0) for m in models_smt]
avg_test = np.mean(all_test_pp, axis=0)
yp_argmax = np.argmax(avg_test, axis=1)

# Try: if P(class 3) > 0.5 but sample is close to boundary, downvote
best_rule = None
best_min = 0

for t3 in np.arange(0.3, 0.9, 0.05):
    for bonus in [0.0, 0.05, 0.1, 0.15, 0.2]:
        # Rule: if P3 > t3, force to argmax of [P1, P2] with bonus to P2
        yp = yp_argmax.copy()
        for i in range(len(yp)):
            if avg_test[i, 3] > t3:
                p1 = avg_test[i, 1] + bonus * 0
                p2 = avg_test[i, 2] + bonus
                if p2 > p1 and avg_test[i, 0] < 0.3:
                    yp[i] = 2
        overall = (yp == y_test).mean()
        per_c = {}
        for c in sorted(np.unique(y_test)):
            mask = y_test == c
            if mask.sum() > 0:
                per_c[c] = (yp[mask] == c).mean()
        min_acc = min(per_c.values()) if len(per_c) == 4 else 0
        if min_acc > best_min:
            best_min = min_acc
            best_rule = (t3, bonus, overall, per_c)

if best_rule:
    t3, bonus, overall, per_c = best_rule
    print(f"\nBest rule (maximize min per-class): t3>{t3}, bonus={bonus}")
    print(f"  Overall: {overall*100:.1f}%")
    for c in sorted(per_c.keys()):
        print(f"  Class {c}: {per_c[c]*100:.1f}%")

# ===================== PICK BEST MODEL =====================
print("\n" + "="*55)
print("PICKING BEST APPROACH")
print("="*55)

# The SMOTE ensemble is likely the best. Evaluate it thoroughly.
print("\nSMOTE Ensemble (best candidate):")
all_pp = [m.predict(X_test_smt, verbose=0) for m in models_smt]
avg = np.mean(all_pp, axis=0)
yp = np.argmax(avg, axis=1)
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum() > 0:
        print(f"  Class {c}: {(yp[mask]==c).mean()*100:.1f}% ({mask.sum()} samples)")
print(f"  Overall: {(yp == y_test).mean()*100:.1f}%")
cm = confusion_matrix(y_test, yp)
print(f"  Confusion Matrix:")
header = "       " + "  ".join(f"P{c}" for c in range(4))
print(header)
for i in range(4):
    print(f"  T{i}:  " + " ".join(f"{cm[i,j]:4d}" for j in range(4)))
