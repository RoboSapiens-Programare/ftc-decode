import os
import random
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import confusion_matrix
import tensorflow as tf
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau

np.random.seed(42)
tf.random.set_seed(42)
random.seed(42)

# 1. Target and Feature Setup — must match ShootAssist.java exactly (6 features)
features = ["target_dist", "angle_error", "vel_x", "vel_y", "omega", "voltage"]
target = "balls_scored"

csv_path = "ftc_shot_data.csv"
if not os.path.exists(csv_path):
    raise FileNotFoundError(
        f"Could not find '{csv_path}'. Make sure to pull it from the Control Hub first!"
    )

df = pd.read_csv(csv_path)

print("Columns found in your CSV:", df.columns.tolist())
print(f"Original raw dataset size: {len(df)} samples.")
class_counts = df[target].value_counts().sort_index()
print(f"Class distribution:\n{class_counts}\n")

# 2. Stratified Train/Test Split FIRST to prevent Data Leakage
df_train_raw, df_test_raw = train_test_split(
    df, test_size=0.20, random_state=42, stratify=df[target]
)

# 3. Data Augmentation applied ONLY to the Training Set
def add_jitter(row, scale_dist=0.05, scale_angle=0.01, scale_voltage=0.1):
    r = row.copy()
    r['target_dist'] += np.random.normal(0, scale_dist)
    r['angle_error'] += np.random.normal(0, scale_angle)
    r['voltage'] += np.random.normal(0, scale_voltage)
    return r

augmented_train_rows = []
for _, row in df_train_raw.iterrows():
    augmented_train_rows.append(row.to_dict())

    mirror_row = row.copy()
    mirror_row['angle_error'] = -row['angle_error']
    mirror_row['vel_y'] = -row['vel_y']
    mirror_row['omega'] = -row['omega']
    augmented_train_rows.append(mirror_row.to_dict())

    for _ in range(3):
        for base_row in [row, mirror_row]:
            augmented_train_rows.append(add_jitter(base_row, 0.05, 0.01, 0.1).to_dict())

for cls, extra_copies in [(1, 16), (2, 8)]:
    cls_rows = df_train_raw[df_train_raw[target] == cls]
    for _, row in cls_rows.iterrows():
        for _ in range(extra_copies):
            augmented_train_rows.append(add_jitter(row, 0.08, 0.02, 0.15).to_dict())

df_train_aug = pd.DataFrame(augmented_train_rows)
aug_class_dist = df_train_aug[target].value_counts().sort_index()
print(f"Final augmented training dataset size: {len(df_train_aug)} samples.")
print(f"Augmented class distribution:\n{aug_class_dist}\n")
print(f"Clean testing baseline dataset size: {len(df_test_raw)} samples.\n")

# Prepare feature matrices and target arrays
X_train = df_train_aug[features].values
y_train = df_train_aug[target].values
X_test = df_test_raw[features].values
y_test = df_test_raw[target].values

# 4. Standardized Scaling Transformation
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

print(f"=== STANDARD SCALER VALUES (MUST update ShootAssist.java EXACTLY) ===")
for i, feat in enumerate(features):
    mean_val = scaler.mean_[i]
    std_val = np.sqrt(scaler.var_[i])
    print(f"  {feat}: mean={mean_val:.3f}, std={std_val:.3f}")
print(f"===============================================================\n")

# Save scaler values to JSON
import json
scaler_data = {
    "features": features,
    "means": [float(scaler.mean_[i]) for i in range(len(features))],
    "stds": [float(np.sqrt(scaler.var_[i])) for i in range(len(features))],
}
with open("scaler_values.json", "w") as f:
    json.dump(scaler_data, f, indent=2)
print("Scaler values saved to scaler_values.json")

# 5. sklearn Baselines
rf = RandomForestClassifier(n_estimators=300, max_depth=8, class_weight='balanced', random_state=42)
rf.fit(X_train, y_train)
rf_pred = rf.predict(X_test)
print(f"\n--- Random Forest Baseline (test set) ---")
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum() > 0:
        acc = (rf_pred[mask] == c).mean()
        print(f"  RF Class {c}: {acc*100:.1f}% ({mask.sum()} samples)")
print(f"  RF Overall: {(rf_pred == y_test).mean()*100:.1f}%")

lr = LogisticRegression(max_iter=1000, class_weight='balanced', random_state=42)
lr.fit(X_train, y_train)
lr_pred = lr.predict(X_test)
print(f"\n--- Logistic Regression Baseline ---")
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum() > 0:
        acc = (lr_pred[mask] == c).mean()
        print(f"  LR Class {c}: {acc*100:.1f}% ({mask.sum()} samples)")
print(f"  LR Overall: {(lr_pred == y_test).mean()*100:.1f}%")

# 6. Cumulative Link Ordinal Model
inputs = tf.keras.Input(shape=(len(features),), name='input_layer')

x = tf.keras.layers.Dense(128, activation='swish')(inputs)
x = tf.keras.layers.Dropout(0.2)(x)
x = tf.keras.layers.Dense(64, activation='swish')(x)
x = tf.keras.layers.Dropout(0.15)(x)
x = tf.keras.layers.Dense(32, activation='swish')(x)

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
model.summary()

model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy']
)

# 7. Callbacks
callbacks = [
    EarlyStopping(monitor='val_loss', mode='min', patience=30, restore_best_weights=True),
    ReduceLROnPlateau(monitor='val_loss', mode='min', factor=0.5, patience=10, min_lr=1e-6)
]

# 8. Model Training
history = model.fit(
    X_train_scaled,
    y_train,
    validation_data=(X_test_scaled, y_test),
    epochs=200,
    batch_size=16,
    callbacks=callbacks,
    verbose=1
)

# 9. Export to Float16 TFLite
print("\nConverting model to optimized Float16 TFLite flatbuffer...")
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.target_spec.supported_types = [tf.float16]
tflite_model = converter.convert()

with open("shoot_predictor.tflite", "wb") as f:
    f.write(tflite_model)
print("Done! Model saved as 'shoot_predictor.tflite'")

# Verify model input/output shapes
interpreter = tf.lite.Interpreter(model_content=tflite_model)
interpreter.allocate_tensors()
in_details = interpreter.get_input_details()[0]
out_details = interpreter.get_output_details()[0]
print(f"TFLite input shape: {in_details['shape']} (expect [1,6])")
print(f"TFLite output shape: {out_details['shape']} (expect [1,4])")
print(f"TFLite input dtype: {in_details['dtype']}")
print(f"TFLite output dtype: {out_details['dtype']}")

# 10. Performance Diagnostics
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(history.history["loss"], label="Training Loss", color="blue")
plt.plot(history.history["val_loss"], label="Validation Loss", color="orange", linestyle="--")
plt.title("Ordinal Model Loss")
plt.xlabel("Epochs")
plt.ylabel("Loss")
plt.legend()
plt.grid(True)

predicted_probs = model.predict(X_test_scaled)
y_pred_classes = np.argmax(predicted_probs, axis=1)

plt.subplot(1, 2, 2)
jitter_x = y_test + np.random.uniform(-0.12, 0.12, len(y_test))
jitter_y = y_pred_classes + np.random.uniform(-0.12, 0.12, len(y_pred_classes))
plt.scatter(jitter_x, jitter_y, alpha=0.45, color="purple")
plt.plot([0, 3], [0, 3], color="red", linestyle=":")
plt.title("Accuracy Alignment")
plt.xticks([0, 1, 2, 3])
plt.yticks([0, 1, 2, 3])
plt.xlabel("Actual Ball Count")
plt.ylabel("Predicted Ball Count")
plt.grid(True)

print("\n--- Test Set Accuracy Per Class ---")
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    if mask.sum() > 0:
        acc = (y_pred_classes[mask] == c).mean()
        print(f"  Class {c}: {acc*100:.1f}% ({mask.sum()} samples)")
print(f"  Overall: {(y_pred_classes == y_test).mean()*100:.1f}%")

cm = confusion_matrix(y_test, y_pred_classes)
print("\n--- Confusion Matrix (rows=true, cols=pred) ---")
header = "        " + "  ".join(f"Pred{c}" for c in sorted(np.unique(y_test)))
print(header)
for i, c in enumerate(sorted(np.unique(y_test))):
    row_str = f"True {c}: " + " ".join(f"{cm[i,j]:5d}" for j in range(cm.shape[1]))
    print(row_str)

plt.tight_layout()
plt.savefig("model_training_performance.png", dpi=200)
print("\nDone! Diagnostics saved to model_training_performance.png")
