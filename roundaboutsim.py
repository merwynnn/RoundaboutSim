import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import sys

# Parameters
n = 20
dt = 0.01
T = 100000

a = 5
b = 1
c = 1

u0 = 2 * math.pi / n
v0 = 1

# Initial angular positions
X = np.array([k * 2 * math.pi / n for k in range(n)])  # use full circle
A = np.zeros((n, n))
for i in range(n - 1):
    A[i, i] = -1
    A[i, i + 1] = 1
A[-1, 0] = 1
A[-1, -1] = -1

B = np.zeros((2*n, 2*n))
B[:n,n:] = np.eye(n)
B[n:,:n] = a*A
B[:n,:n] = b*A-c*np.eye(n)

valeurs_propres, vecteurs_propres = np.linalg.eig(B)

print("Valeurs propres :", valeurs_propres)

U = A @ X
U[-1] += 2 * math.pi

Xv = np.zeros(n)
Xv[0] = 0.5
Xa = np.zeros(n)

# --- RANDOM COLORS (each point keeps its own color) ---
colors = np.random.rand(n, 3)  # n random RGB triplets between 0 and 1

# Live plot setup
plt.ion()
fig, (ax, ax_density) = plt.subplots(1, 2, figsize=(14, 6))

# Left plot: Circle with cars
ax.set_aspect('equal')
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_title("Cars on Circle")

circle = plt.Circle((0, 0), 1, color='lightgray', fill=False)
ax.add_artist(circle)

# Scatter for cars
points = ax.scatter(np.cos(X), np.sin(X), c=colors, s=50)

# Right plot: Density visualization
ax_density.set_xlim(0, 2*math.pi)
ax_density.set_ylim(0, 5)
ax_density.set_xlabel("Angular Position (rad)")
ax_density.set_ylabel("Car Density")
ax_density.set_title("Traffic Density Waves")

# Create bars for density histogram
density_bins = 36  # 36 bins = 10 degrees each
bar_container = ax_density.bar(np.linspace(0, 2*math.pi, density_bins), 
                               np.zeros(density_bins), width=2*math.pi/density_bins)

# --- SPEEDUP PARAMETERS ---
frame_skip = 5
pause_time = dt

# Pre-draw background (for blitting)
fig.canvas.draw()
background = fig.canvas.copy_from_bbox(fig.bbox)

for t in range(T):
    # Dynamics
    Xa = a * (U - u0) + b * A @ Xv - c * (Xv - v0)
    Xv += Xa * dt
    
    # --- PREVENT BACKWARD MOTION ---
    Xv = np.maximum(Xv, 0)  # Clamp velocity to non-negative values
    
    X += Xv * dt
    U = A @ X
    U[-1] += 2 * math.pi

    # Update every few steps only
    if t % frame_skip == 0:
        fig.canvas.restore_region(background)
        
        # Update car positions on circle
        points.set_offsets(np.c_[np.cos(X), np.sin(X)])
        ax.draw_artist(points)
        
        # --- UPDATE DENSITY VISUALIZATION ---
        # Normalize angles to [0, 2π)
        X_normalized = X % (2 * math.pi)
        
        # Calculate density in each bin
        density = np.zeros(density_bins)
        bin_width = 2 * math.pi / density_bins
        for angle in X_normalized:
            bin_idx = int(angle / bin_width) % density_bins
            density[bin_idx] += 1
        
        # Update bar heights
        for i, bar in enumerate(bar_container):
            bar.set_height(density[i])
        
        ax_density.draw_artist(ax_density.patches[0])  # redraw bars
        for bar in bar_container:
            ax_density.draw_artist(bar)
        
        fig.canvas.blit(fig.bbox)
        fig.canvas.flush_events()

    # Allow clean exit with Ctrl+C
    try:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
        break

plt.ioff()
plt.show()
