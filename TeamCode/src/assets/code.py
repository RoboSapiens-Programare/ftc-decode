import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score

# 1. Load and parse the data
file_path = "data.txt"  # Replace with your actual file name

try:
    df = pd.read_csv(file_path, sep=r'\s*\|\s*', names=['x', 'y'], engine='python', header=None)
except Exception as e:
    print(f"Error reading file: {e}")
    exit()

# Clean up any missing data
df = df.dropna()
x_data = df['x'].values
y_data = df['y'].values

# Sort data for clean plotting line generation
sort_idx = np.argsort(x_data)
x_plot = x_data[sort_idx]
y_plot = y_data[sort_idx]

# Define the models we want to fit (degree mapping)
models = {
    "Linear (1st Degree)": 1,
    "Quadratic (2nd Degree)": 2,
    "Cubic (3rd Degree)": 3,
    "Quartic (4th Degree)": 4,
    "Polynomial (5th Degree)": 5,
    "Polynomial (6th Degree)": 6
}

# Setup the plot
plt.figure(figsize=(12, 8))
plt.scatter(x_data, y_data, color='black', alpha=0.5, label='Data Points', zorder=2)

print("--- Regression Results & R² Scores ---")
print(f"{'Model Type':<25} | {'R² Score':<10}")
print("-" * 40)

# Colors for the plotting lines
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#1f00f0']

# 2. Compute regressions, evaluate R², and plot
for (name, degree), color in zip(models.items(), colors):
    # Fit the polynomial coefficients
    coefficients = np.polyfit(x_data, y_data, degree)
    
    # Create a polynomial function from coefficients
    poly_func = np.poly1d(coefficients)
    
    # Calculate predictions on the original dataset to evaluate R²
    y_pred = poly_func(x_data)
    r2 = r2_score(y_data, y_pred)
    
    print(f"{name:<25} | {r2:.6f}")
    terms = []
    for i, c in enumerate(coefficients):
        power = len(coefficients) - 1 - i
        if power == 0:
            terms.append(f"{c:.10f}")
        elif power == 1:
            terms.append(f"{c:.10f} * distance")
        else:
            terms.append(f"{c:.10f} * Math.pow(distance, {power})")
    poly_str = " + ".join(terms)
    print(f"  double result = {poly_str};")
    
    # Generate smooth lines for plotting
    x_smooth = np.linspace(x_plot.min(), x_plot.max(), 300)
    y_smooth = poly_func(x_smooth)
    
    # Plot the regression line
    plt.plot(x_smooth, y_smooth, color=color, linewidth=2.5, label=f'{name} ($R^2$: {r2:.4f})')

# 3. Finalize visualization
plt.title('Comparison of Multiple Regression Models', fontsize=14, fontweight='bold')
plt.xlabel('X', fontsize=12)
plt.ylabel('Y', fontsize=12)
plt.legend(loc='best', fontsize=10)
plt.grid(True, linestyle='--', alpha=0.5)
plt.tight_layout()
plt.show()
