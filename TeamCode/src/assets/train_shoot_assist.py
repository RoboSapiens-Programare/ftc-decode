import os
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from tensorflow.keras.callbacks import EarlyStopping, ReduceLROnPlateau
import tensorflow as tf

# 1. Target and Feature Setup (Moved to top to prevent NameErrors)
features = ["target_dist", "angle_error", "vel_x", "vel_y", "omega", "voltage"]
target = "balls_scored"

csv_path = "ftc_shot_data.csv"
if not os.path.exists(csv_path):
    raise FileNotFoundError(
        f"Could not find '{csv_path}'. Make sure to pull it from the Control Hub first!"
    )

df = pd.read_csv(csv_path)

# --- QUICK DIAGNOSTIC PRINT ---
print("Columns found in your CSV:", df.columns.tolist())

# --- CLASS DISTRIBUTION DIAGNOSTIC ---
print(f"Original raw dataset size: {len(df)} samples.")
class_counts = df[target].value_counts().sort_index()
print(f"Class distribution:\n{class_counts}")

# --- DATA AUGMENTATION ON ALL DATA ---
# Every sample gets mirror symmetry + jitter (8x base expansion).
# Class 0 (only 4 real samples) gets extra oversampling via more jitter.
augmented_rows = []
for _, row in df.iterrows():
    augmented_rows.append(row.to_dict())

    mirror_row = row.copy()
    mirror_row['angle_error'] = -row['angle_error']
    mirror_row['vel_y'] = -row['vel_y']
    mirror_row['omega'] = -row['omega']
    augmented_rows.append(mirror_row.to_dict())

    for _ in range(3):
        for base_row in [row, mirror_row]:
            jitter_row = base_row.copy()
            jitter_row['target_dist'] += np.random.normal(0, 0.05)
            jitter_row['angle_error'] += np.random.normal(0, 0.01)
            jitter_row['voltage'] += np.random.normal(0, 0.1)
            augmented_rows.append(jitter_row.to_dict())

# Extra oversampling for class 0 (only 4 real samples → needs more diversity)
class_0_rows = df[df[target] == 0]
for _, row in class_0_rows.iterrows():
    for _ in range(8):
        jitter_row = row.copy()
        jitter_row['target_dist'] += np.random.normal(0, 0.08)
        jitter_row['angle_error'] += np.random.normal(0, 0.02)
        jitter_row['voltage'] += np.random.normal(0, 0.15)
        augmented_rows.append(jitter_row.to_dict())

df = pd.DataFrame(augmented_rows)
print(f"Final augmented dataset size: {len(df)} samples total.\n")

# Prepare matrices
X = df[features].values
y = df[target].values

# Mild class weights (sqrt-scaled, clipped) to avoid destabilizing the loss
classes = np.sort(np.unique(y))
n_total = len(y)
raw_weights = {c: n_total / (len(classes) * (y == c).sum()) for c in classes}
max_w = max(raw_weights.values())
class_weight_dict = {
    c: max(0.5, min(5.0, w / max_w * 3.0))
    for c, w in raw_weights.items()
}
print(f"Class weights (mild): {class_weight_dict}\n")

# 2. Stratified Train/Test Split (preserves class ratios in both sets)
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.20, random_state=42, stratify=y
)

# 3. Fit and Apply Scaling Transforms
scaler = StandardScaler()
X_train_scaled = scaler.fit_transform(X_train)
X_test_scaled = scaler.transform(X_test)

print("Mean values for scaling:", scaler.mean_)
print("Variance values for scaling:", scaler.var_)

# 4. Neural Network Architecture
model = tf.keras.Sequential([
    tf.keras.Input(shape=(len(features),)),

    tf.keras.layers.Dense(128, activation='swish'),
    tf.keras.layers.Dropout(0.2),

    tf.keras.layers.Dense(64, activation='swish'),
    tf.keras.layers.Dropout(0.15),

    tf.keras.layers.Dense(32, activation='swish'),
    tf.keras.layers.Dropout(0.1),

    tf.keras.layers.Dense(4, activation='softmax')
])

# 5. Compile
model.compile(
    optimizer=tf.keras.optimizers.Adam(learning_rate=0.001),
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy']
)

# 6. Training Optimization Callbacks
callbacks = [
    EarlyStopping(monitor='val_loss', patience=30, restore_best_weights=True),
    ReduceLROnPlateau(monitor='val_loss', factor=0.5, patience=10, min_lr=1e-6)
]

# 7. Train with Class Weights for Imbalanced Data
history = model.fit(
    X_train_scaled, y_train,
    validation_data=(X_test_scaled, y_test),
    epochs=200,
    batch_size=16,
    class_weight=class_weight_dict,
    callbacks=callbacks,
    verbose=1
)

# 8. Export to TFLite flatbuffer format
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()

with open("shoot_predictor.tflite", "wb") as f:
    f.write(tflite_model)
print("\nDone! Model successfully saved as 'shot_predictor.tflite'")

# 9. Generate Performance Charts
plt.figure(figsize=(12, 5))

# Plot 1: Categorical Loss Curves
plt.subplot(1, 2, 1)
plt.plot(history.history["loss"], label="Training Loss (80%)", color="blue")
plt.plot(
    history.history["val_loss"],
    label="Validation Loss (20%)",
    color="orange",
    linestyle="--",
)
plt.title("Model Loss Optimization (Crossentropy)")
plt.xlabel("Epochs")
plt.ylabel("Loss Error")
plt.legend()
plt.grid(True)

# Plot 2: Confusion Matrix
raw_predictions = model.predict(X_test_scaled)
y_pred_classes = np.argmax(raw_predictions, axis=1)

plt.subplot(1, 2, 2)
jitter_x = y_test + np.random.uniform(-0.1, 0.1, len(y_test))
jitter_y = y_pred_classes + np.random.uniform(-0.1, 0.1, len(y_pred_classes))

plt.scatter(jitter_x, jitter_y, alpha=0.4, color="purple")
plt.plot([0, 3], [0, 3], color="red", linestyle=":")
plt.title("Prediction Accuracy Check")
plt.xticks([0, 1, 2, 3])
plt.yticks([0, 1, 2, 3])
plt.xlabel("Actual Ball Count")
plt.ylabel("Predicted Ball Count")
plt.grid(True)

# Per-class accuracy
print("\n--- Per-Class Accuracy ---")
for c in sorted(np.unique(y_test)):
    mask = y_test == c
    acc = (y_pred_classes[mask] == c).mean()
    print(f"  Class {c}: {acc*100:.1f}% ({mask.sum()} samples)")
print(f"  Overall: {(y_pred_classes == y_test).mean()*100:.1f}%")

plt.tight_layout()
plt.savefig("model_training_performance.png")
plt.show()