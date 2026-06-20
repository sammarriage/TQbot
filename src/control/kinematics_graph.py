import matplotlib.pyplot as plt
import numpy as np
from tqbot_kinematics import LegKinematics, Leg

# 1. Define the angle sets
ROUND_TRIP_ANGLES = [
    (0.0, 0.80, -0.50), (0.0, 1.20, -0.60), (0.0, 0.30, -0.10),
    (0.0, 0.70, -1.10), (0.0, 0.50, -0.90), (0.0, 0.20, -0.30),
    (-0.1, 0.0, -0.50), (0.2, 0.0, -0.60), (0.15, 0.0, -0.10),
    (-0.2, 0.70, 0.0), (0.5, 0.50, 0.0), (0.3, 0.20, 0.0),
    (0.30, 0.90, -0.45), (-0.30, 0.90, -0.45), (0.15, 1.10, -0.55),
    (-0.15, 1.10, -0.55), (0.20, 0.65, -0.30), (-0.20, 0.75, -0.35),
    (0.50, 0.80, -0.50),
]

# 2. Setup and data collection
kin = LegKinematics(Leg.FRONT_LEFT)

calc_pts = {'x': [], 'y': [], 'z': []}
residuals = []

for t1, t2, t3 in ROUND_TRIP_ANGLES:
    # Target Position (Forward Kinematics) required for error calculation
    px, py, pz = kin.forward(t1, t2, t3)

    # Calculated Returned Position (Inverse then Forward Kinematics)
    rt1, rt2, rt3 = kin.inverse(px, py, pz)
    rx, ry, rz = kin.forward(rt1, rt2, rt3)

    # Residual error distance
    res = np.sqrt((px - rx)**2 + (py - ry)**2 + (pz - rz)**2)

    # Append calculated coordinates
    calc_pts['x'].append(rx)
    calc_pts['y'].append(ry)
    calc_pts['z'].append(rz)

    residuals.append(res)

# 3. Initialize Figure with a 2x2 grid
fig = plt.figure(figsize=(16, 12))
plt.subplots_adjust(hspace=0.3, wspace=0.2)

# --- Subplot 1: 3D View ---
ax1 = fig.add_subplot(221, projection='3d')
sc3d = ax1.scatter(calc_pts['x'], calc_pts['y'], calc_pts['z'],
                   c=residuals, cmap='RdYlGn_r', s=80, edgecolors='k')

ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D View')

# Helper function to plot 2D planes to avoid repeating code
def plot_2d_plane(subplot_index, x_axis, y_axis, xlabel, ylabel, title):
    ax = fig.add_subplot(subplot_index)
    sc = ax.scatter(calc_pts[x_axis], calc_pts[y_axis],
                    c=residuals, cmap='RdYlGn_r', s=80, edgecolors='k')

    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, linestyle=':', alpha=0.6)
    return sc

# --- Subplot 2: XY Plane (Top-Down View) ---
plot_2d_plane(222, 'x', 'y', 'X (m)', 'Y (m)', 'XY Plane (Top View)')

# --- Subplot 3: XZ Plane (Side View) ---
plot_2d_plane(223, 'x', 'z', 'X (m)', 'Z (m)', 'XZ Plane (Side View)')

# --- Subplot 4: YZ Plane (Front/Back View) ---
plot_2d_plane(224, 'y', 'z', 'Y (m)', 'Z (m)', 'YZ Plane (Front View)')

# 4. Add a single colourbar
cbar_ax = fig.add_axes([0.92, 0.15, 0.02, 0.7]) # [left, bottom, width, height]
cbar = fig.colorbar(sc3d, cax=cbar_ax)
cbar.set_label('Residual Magnitude (m)')

plt.suptitle('Calculated Foot Position Workspace', fontsize=16)
plt.show()