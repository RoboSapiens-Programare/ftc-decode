import os
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.preprocessing import StandardScaler

# 1. Load raw data and model
csv_path = "ftc_shot_data.csv"
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"Could not find '{csv_path}'. Check your path!")

df = pd.read_csv(csv_path)
df.columns = df.columns.str.strip()

# Reconstruct the exact scaler setup used during model training
# (Assumes your features match the training pipeline)
features = ["target_dist", "angle_error", "vel_x", "vel_y", "omega", "voltage"]
scaler = StandardScaler()
scaler.fit(df[features].values)

# Load your compiled keras model
model_path = "shoot_predictor.tflite" # Or load the standard keras model if saved
# If using the Keras model object directly:
# model = tf.keras.models.load_model('shot_model.h5')

# 2. Layout & Bins Configuration
bin_size = 0.20  # Meters
max_dist = df['target_dist'].max()
distance_bins = np.arange(0, max_dist + bin_size, bin_size)

theta_start = np.radians(-90)
theta_width = np.radians(180)

# Custom Colormap Setup (Red -> Yellow -> Green)
colors = ["#d62728", "#fded11", "#2ca02c"]
custom_cmap = mcolors.LinearSegmentedColormap.from_list("RedYellowGreen", colors)
norm = mcolors.Normalize(vmin=0, vmax=3)

fig = plt.figure(figsize=(16, 8))
fig.patch.set_facecolor('#f0f0f0')

# ==========================================
# LEFT PLOT: REAL WORLD LOGGED TELEMETRY
# ==========================================
ax_left = plt.subplot(1, 2, 1, projection='polar')
ax_left.set_facecolor('black')

for i in range(len(distance_bins) - 1):
    r_inner = distance_bins[i]
    r_outer = distance_bins[i+1]
    r_height = r_outer - r_inner

    mask = (df['target_dist'] >= r_inner) & (df['target_dist'] < r_outer)
    matching_shots = df[mask]

    if len(matching_shots) > 0:
        avg_score = matching_shots['balls_scored'].mean()
        color = custom_cmap(norm(avg_score))

        ax_left.bar(
            x=theta_start + (theta_width / 2),
            height=r_height,
            width=theta_width,
            bottom=r_inner,
            color=color,
            edgecolor=color,
            linewidth=0.5
        )

ax_left.set_thetamin(-90)
ax_left.set_thetamax(90)
ax_left.set_theta_zero_location('N')
ax_left.set_ylim(0, max_dist + 0.2)
ax_left.grid(True, color='#ffffff', linestyle=':', linewidth=1.0, alpha=0.4)
ax_left.set_title("Actual Logged Telemetry\n(CSV Real-World Average)", fontsize=13, fontweight='bold', pad=15)

# ==========================================
# RIGHT PLOT: NEURAL NETWORK PREDICTIONS
# ==========================================
ax_right = plt.subplot(1, 2, 2, projection='polar')
ax_right.set_facecolor('black')

# If using TFLite interpreter instead of native keras object:
interpreter = tf.lite.Interpreter(model_path="shoot_predictor.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

for i in range(len(distance_bins) - 1):
    r_inner = distance_bins[i]
    r_outer = distance_bins[i+1]
    r_center = (r_inner + r_outer) / 2.0
    r_height = r_outer - r_inner

    # Create an ideal testing vector profile for this distance step:
    # [distance, perfect angle error, 0 velocity, 0 strafe, 0 omega, nominal 13.0V battery]
    ideal_sample = np.array([[r_center, 0.0, 0.0, 0.0, 0.0, 13.0]])

    # Apply Standard Scaler mapping transformation
    scaled_sample = scaler.transform(ideal_sample).astype(np.float32)

    # Run Inference via TFLite Interpreter
    interpreter.set_tensor(input_details[0]['index'], scaled_sample)
    interpreter.invoke()
    raw_probabilities = interpreter.get_tensor(output_details[0]['index'])[0]

    # Calculate Expected Ball Value: E[X] = Sum(class * probability)
    predicted_yield = (0 * raw_probabilities[0]) + (1 * raw_probabilities[1]) + (2 * raw_probabilities[2]) + (3 * raw_probabilities[3])

    # Color the ring based on what the network predicts would happen here
    color = custom_cmap(norm(predicted_yield))

    ax_right.bar(
        x=theta_start + (theta_width / 2),
        height=r_height,
        width=theta_width,
        bottom=r_inner,
        color=color,
        edgecolor=color,
        linewidth=0.5
    )

ax_right.set_thetamin(-90)
ax_right.set_thetamax(90)
ax_right.set_theta_zero_location('N')
ax_right.set_ylim(0, max_dist + 0.2)
ax_right.grid(True, color='#ffffff', linestyle=':', linewidth=1.0, alpha=0.4)
ax_right.set_title("Neural Network Expected Yield\n(Model Prediction Profile)", fontsize=13, fontweight='bold', pad=15)

# 3. Add Shared Colorbar Indicator
sm = plt.cm.ScalarMappable(cmap=custom_cmap, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=[ax_left, ax_right], orientation='horizontal', pad=0.1, shrink=0.6)
cbar.set_label('Scoring Yield Evaluation (0 to 3 Balls)', fontsize=12, fontweight='bold')
cbar.set_ticks([0, 1, 2, 3])
cbar.ax.set_xticklabels(['0 (Miss)', '1 Ball', '2 Balls', '3 Balls (Perfect)'])

plt.suptitle("Side-by-Side Target Range Comparison", fontsize=16, fontweight='bold', y=0.96)
plt.savefig("telemetry_vs_model_prediction.png", dpi=300, facecolor=fig.get_facecolor())
plt.show()