import numpy as np
import matplotlib.pyplot as plt

# Define the map dimensions (50 rows x 180 columns)
map_height = 3000
map_width = 5400

# Create a grid for visualization
x = np.linspace(0, map_width, 500)
y = np.linspace(0, map_height, 500)
X, Y = np.meshgrid(x, y)

# Define the shapes using semi-algebraic models
def vertical_lines (x, y):
    vertical_1 = (1000 <= x) & (x <= 1100) & (10 <= y) & (y <= 2010)
    vertical_2 = (2100 <= x) & (x <= 2200) & (990 <= y) & (y <= 2990)
    vertical_3 = (3200 <= x) & (x <= 3300) & (1990 <= y) & (y <= 2990)
    vertical_4 = (3200 <= x) & (x <= 3300) & (10 <= y) & (y <= 1010)
    vertical_5 = (4300 <= x) & (x <= 4400) & (10 <= y) & (y <= 2010)
    vertical_border_right = (0 <= x) & (x <= 10) & (0 <= y) & (y <= 3000)
    vertical_border_left = (5390 <= x) & (x <= 5400) & (0 <= y) & (y <= 3000)
    horizontal_border_top = (0 <= x) & (x <= 5400) & (2990 <= y) & (y <= 3000)
    horizontal_border_bottom = (0 <= x) & (x <= 5400) & (0 <= y) & (y <= 10)
    return vertical_1 | vertical_2 | vertical_3 | vertical_4 | vertical_5 | vertical_border_right | vertical_border_left | horizontal_border_top | horizontal_border_bottom

# Plot the shapes
plt.figure(figsize=(18, 5))
plt.imshow(vertical_lines(X,Y), extent=(0, map_width, 0, map_height), origin='lower', cmap='binary', alpha=0.8)
plt.title("Map with Obstacles")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
plt.show()

