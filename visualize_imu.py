import serial
import re
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PORT = '/dev/cu.usbserial-0001'
BAUD = 115200

roll_deg = 0.0
pitch_deg = 0.0
yaw_deg = 0.0

ser = serial.Serial(PORT, BAUD, timeout=1)
ser.reset_input_buffer()

def _read_serial():
    global roll_deg, pitch_deg, yaw_deg
    while True:
        try:
            line = ser.readline().decode('latin-1')
            m = re.search(r'roll:([-\d.]+)\s+pitch:([-\d.]+)\s+yaw:([-\d.]+)', line)
            if m:
                roll_deg  = float(m.group(1))
                pitch_deg = float(m.group(2))
                yaw_deg   = float(m.group(3))
        except Exception:
            pass

threading.Thread(target=_read_serial, daemon=True).start()

fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('#1e1e1e')
ax.set_facecolor('#1e1e1e')

title = fig.suptitle('roll: 0.0°   pitch: 0.0°   yaw: 0.0°', color='white', fontsize=14)

# Set up axis once — never cleared again
ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_zlim(-1.2, 1.2)
ax.set_xlabel('X', color='gray')
ax.set_ylabel('Y', color='gray')
ax.set_zlabel('Z', color='gray')
ax.tick_params(colors='gray')
for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
    pane.fill = False
    pane.set_edgecolor('gray')

# Static horizon ring drawn once
theta = np.linspace(0, 2*np.pi, 60)
ax.plot(np.cos(theta)*1.1, np.sin(theta)*1.1, 0,
        color='#444444', linewidth=0.8, linestyle='--')

_dynamic_artists = []

def board_vertices(roll_r, pitch_r, yaw_r):
    w, h, t = 1.6, 1.0, 0.05
    corners = np.array([
        [-w/2, -h/2, 0], [ w/2, -h/2, 0],
        [ w/2,  h/2, 0], [-w/2,  h/2, 0],
        [-w/2, -h/2, t], [ w/2, -h/2, t],
        [ w/2,  h/2, t], [-w/2,  h/2, t],
    ], dtype=float)

    cp, sp = np.cos(pitch_r), np.sin(pitch_r)
    cr, sr = np.cos(roll_r),  np.sin(roll_r)
    cy, sy = np.cos(yaw_r),   np.sin(yaw_r)

    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])   # roll around X
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])   # pitch around Y
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])   # yaw around Z
    R  = Rz @ Ry @ Rx

    return (R @ corners.T).T

def make_faces(v):
    return [
        [v[0],v[1],v[5],v[4]],  # front
        [v[2],v[3],v[7],v[6]],  # back
        [v[0],v[3],v[7],v[4]],  # left
        [v[1],v[2],v[6],v[5]],  # right
        [v[4],v[5],v[6],v[7]],  # top  (green)
        [v[0],v[1],v[2],v[3]],  # bottom
    ]

def update(_):
    global _dynamic_artists
    for artist in _dynamic_artists:
        artist.remove()
    _dynamic_artists = []

    v = board_vertices(np.radians(roll_deg), np.radians(pitch_deg), np.radians(yaw_deg))
    faces = make_faces(v)
    colors = ['#4a90d9','#4a90d9','#4a90d9','#4a90d9','#2ecc71','#c0392b']
    alphas = [0.7,       0.7,       0.7,       0.7,       0.9,      0.6    ]
    for f, c, a in zip(faces, colors, alphas):
        p = Poly3DCollection([f], facecolors=[c], edgecolors='white', linewidths=0.4)
        p.set_alpha(a)
        ax.add_collection3d(p)
        _dynamic_artists.append(p)

    yr, pr, rr = np.radians(yaw_deg), np.radians(pitch_deg), np.radians(roll_deg)
    fwd = np.array([0.5, 0, 0.025])                                                        # nose along +X
    R_roll  = np.array([[1,0,0],[0,np.cos(rr),-np.sin(rr)],[0,np.sin(rr),np.cos(rr)]])   # roll around X
    R_pitch = np.array([[np.cos(pr),0,np.sin(pr)],[0,1,0],[-np.sin(pr),0,np.cos(pr)]])   # pitch around Y
    R_yaw   = np.array([[np.cos(yr),-np.sin(yr),0],[np.sin(yr),np.cos(yr),0],[0,0,1]])   # yaw around Z
    fwd_rot = R_yaw @ R_pitch @ R_roll @ fwd
    q = ax.quiver(0, 0, 0, fwd_rot[0], fwd_rot[1], fwd_rot[2],
                  color='yellow', linewidth=2, arrow_length_ratio=0.3)
    _dynamic_artists.append(q)

    title.set_text(f'roll: {roll_deg:+.1f}°   pitch: {pitch_deg:+.1f}°   yaw: {yaw_deg:+.1f}°')

ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout()
plt.show()

ser.close()
