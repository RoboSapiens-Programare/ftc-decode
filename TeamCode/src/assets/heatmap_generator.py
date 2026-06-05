import os
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np
import pandas as pd
import tensorflow as tf
from sklearn.preprocessing import StandardScaler
from scipy.interpolate import LinearNDInterpolator

# 1. Load raw data and model
csv_path = "ftc_shot_data.csv"
if not os.path.exists(csv_path):
    raise FileNotFoundError(f"Could not find '{csv_path}'. Check your path!")

df = pd.read_csv(csv_path)
df.columns = df.columns.str.strip()

# Setup Scaler
features = ["target_dist", "angle_error", "vel_x", "vel_y", "omega", "voltage"]
scaler = StandardScaler()
scaler.fit(df[features].values)

# 2. Config Bins
bin_size_dist = 0.20
max_dist = df['target_dist'].max()
distance_bins = np.arange(0, max_dist + bin_size_dist, bin_size_dist)

# Broad angle segments to prevent visual scattering
bin_size_deg = 30.0
angle_bins_deg = np.arange(-90, 90 + bin_size_deg, bin_size_deg)
angle_bins_rad = np.radians(angle_bins_deg)

# Custom Bright Colormap Setup (Red -> Yellow -> Vibrant Green)
colors = ["#e63946", "#ffb703", "#2a9d8f"]
custom_cmap = mcolors.LinearSegmentedColormap.from_list("VibrantHeatmap", colors)
norm = mcolors.Normalize(vmin=0, vmax=3)

# Setup 3-Subplot Figure
fig = plt.figure(figsize=(22, 8))
fig.patch.set_facecolor('#f5f5f5')

# ==========================================
# PRE-CALCULATE INTERPOLATION FOR PLOT 2
# ==========================================
known_points = df[['target_dist', 'angle_error']].values
known_values = df['balls_scored'].values
linear_interp = LinearNDInterpolator(known_points, known_values)

def get_interpolated_yield(r, theta):
    val = linear_interp(r, theta)
    if np.isnan(val):
        distances = np.sqrt((known_points[:, 0] - r)**2 + (known_points[:, 1] - theta)**2)
        nearest_indices = np.argsort(distances)[:2]
        val = np.mean(known_values[nearest_indices])
    return val

# ==========================================
# LEFT PLOT: REAL WORLD LOGGED TELEMETRY
# ==========================================
ax_left = plt.subplot(1, 3, 1, projection='polar')
ax_left.set_facecolor('black')  # Keep black here to let sparse high-intensity points stand out

for i in range(len(distance_bins) - 1):
    r_inner = distance_bins[i]
    r_outer = distance_bins[i+1]
    r_height = r_outer - r_inner

    for j in range(len(angle_bins_rad) - 1):
        theta_inner = angle_bins_rad[j]
        theta_outer = angle_bins_rad[j+1]
        theta_width = theta_outer - theta_inner

        mask = (
            (df['target_dist'] >= r_inner) & (df['target_dist'] < r_outer) &
            (df['angle_error'] >= theta_inner) & (df['angle_error'] < theta_outer)
        )
        matching_shots = df[mask]

        if len(matching_shots) > 0:
            avg_score = matching_shots['balls_scored'].mean()
            ax_left.bar(theta_inner, r_height, width=theta_width, bottom=r_inner,
                        color=custom_cmap(norm(avg_score)), edgecolor='none', align='edge')

# ==========================================
# CENTER PLOT: ARITHMETIC INTERPOLATION
# ==========================================
ax_center = plt.subplot(1, 3, 2, projection='polar')
ax_center.set_facecolor('#fcfcfc')  # Light background prevents grid bleeding

for i in range(len(distance_bins) - 1):
    r_inner = distance_bins[i]
    r_outer = distance_bins[i+1]
    r_center = (r_inner + r_outer) / 2.0
    r_height = r_outer - r_inner

    for j in range(len(angle_bins_rad) - 1):
        theta_inner = angle_bins_rad[j]
        theta_outer = angle_bins_rad[j+1]
        theta_center = (theta_inner + theta_outer) / 2.0
        theta_width = theta_outer - theta_inner

        interp_yield = get_interpolated_yield(r_center, theta_center)

        ax_center.bar(theta_inner, r_height, width=theta_width, bottom=r_inner,
                      color=custom_cmap(norm(interp_yield)), edgecolor='none', align='edge')

# ==========================================
# RIGHT PLOT: NEURAL NETWORK PREDICTIONS
# ==========================================
ax_right = plt.subplot(1, 3, 3, projection='polar')
ax_right.set_facecolor('#fcfcfc')  # Light background for a clean, professional surface profile

interpreter = tf.lite.Interpreter(model_path="shoot_predictor.tflite")
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

for i in range(len(distance_bins) - 1):
    r_inner = distance_bins[i]
    r_outer = distance_bins[i+1]
    r_center = (r_inner + r_outer) / 2.0
    r_height = r_outer - r_inner

    for j in range(len(angle_bins_rad) - 1):
        theta_inner = angle_bins_rad[j]
        theta_outer = angle_bins_rad[j+1]
        theta_center = (theta_inner + theta_outer) / 2.0
        theta_width = theta_outer - theta_inner

        ideal_sample = np.array([[r_center, theta_center, 0.0, 0.0, 0.0, 13.0]])
        scaled_sample = scaler.transform(ideal_sample).astype(np.float32)

        interpreter.set_tensor(input_details[0]['index'], scaled_sample)
        interpreter.invoke()
        raw_probabilities = interpreter.get_tensor(output_details[0]['index'])[0]

        predicted_yield = (0 * raw_probabilities[0]) + (1 * raw_probabilities[1]) + (2 * raw_probabilities[2]) + (3 * raw_probabilities[3])

        ax_right.bar(theta_inner, r_height, width=theta_width, bottom=r_inner,
                     color=custom_cmap(norm(predicted_yield)), edgecolor='none', align='edge')

# Formatting Configurations
for ax, title, is_dark in zip([ax_left, ax_center, ax_right],
                             ["1. Raw Logged Telemetry\n(Sparse Data)",
                              "2. Interpolated Telemetry\n(Arithmetic Blended)",
                              "3. NN Expected Yield\n(Prediction Profile)"],
                             [True, False, False]):
    ax.set_thetamin(-90)
    ax.set_thetamax(90)
    ax.set_theta_zero_location('N')
    ax.set_ylim(0, max_dist + 0.2)
    grid_color = '#ffffff' if is_dark else '#7f8c8d'
    ax.grid(True, color=grid_color, linestyle=':', linewidth=0.8, alpha=0.4)
    ax.set_title(title, fontsize=12, fontweight='bold', pad=15)

# Colorbar Layout Fixing
sm = plt.cm.ScalarMappable(cmap=custom_cmap, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=[ax_left, ax_center, ax_right], orientation='horizontal', pad=0.12, shrink=0.5)
cbar.set_label('Scoring Yield Evaluation (Expected Balls Hit)', fontsize=11, fontweight='bold')
cbar.set_ticks([0, 1, 2, 3])
cbar.ax.set_xticklabels(['0 (Miss)', '1 Ball', '2 Balls', '3 Balls (Perfect)'])

plt.suptitle("High-Contrast Spatial Accuracy Analysis Map", fontsize=16, fontweight='bold', y=0.98)
plt.savefig("telemetry_triple_comparison_bright.png", dpi=300, facecolor=fig.get_facecolor())
plt.show()