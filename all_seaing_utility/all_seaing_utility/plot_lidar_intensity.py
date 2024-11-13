import matplotlib.pyplot as plt
import pandas as pd

# Load the data
data = pd.read_csv("xy_intensity.csv")

# Extract x, y, and intensity
x = data['x']
y = data['y']
intensity = data['intensity']

# Create a scatter plot
plt.figure(figsize=(10, 8))
scatter = plt.scatter(x, y, c=intensity, cmap='viridis', s=2)  # 'viridis' colormap for intensity
plt.colorbar(scatter, label='Intensity')  # Add a color bar for intensity
plt.title("Point Cloud (x, y) with Intensity")
plt.xlabel("X")
plt.ylabel("Y")
plt.axis("equal")  # Equal aspect ratio
plt.grid(True)
plt.show()

