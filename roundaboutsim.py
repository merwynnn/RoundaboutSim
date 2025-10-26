import numpy as np
import math
import matplotlib.pyplot as plt
import sys

# Parameters
n = 20
dt = 0.01
T = 100000

a = 2
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
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_title("Points moving on a unit circle")

circle = plt.Circle((0, 0), 1, color='lightgray', fill=False)
ax.add_artist(circle)

# Instead of a single Line2D, use a scatter for multiple colors
points = ax.scatter(np.cos(X), np.sin(X), c=colors, s=50)

# --- SPEEDUP PARAMETERS ---
frame_skip = 5
pause_time = dt

# Pre-draw background (for blitting)
fig.canvas.draw()
background = fig.canvas.copy_from_bbox(ax.bbox)

for t in range(T):
    # Dynamics
    Xa = a * (U - u0) + b * A @ Xv - c * (Xv - v0)
    Xv += Xa * dt
    X += Xv * dt
    U = A @ X
    U[-1] += 2 * math.pi

    # Update every few steps only
    if t % frame_skip == 0:
        fig.canvas.restore_region(background)
        points.set_offsets(np.c_[np.cos(X), np.sin(X)])
        ax.draw_artist(points)
        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()

    # Allow clean exit with Ctrl+C
    try:
        pass
    except KeyboardInterrupt:
        sys.exit(0)
        break

plt.ioff()
plt.show()
