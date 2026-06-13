import serial
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

PORT = '/dev/cu.usbserial-0001'
BAUD = 115200

# Display-only orientation conventions. These map the firmware's reported angles
# to the on-screen board so motion matches the physical board. Flip any sign if
# that axis appears reversed:
#   FORWARD_SIGN  -> nose/tail (swaps front/back only)
#   ROLL_SIGN     -> left/right banking
#   PITCH_SIGN    -> nose up/down
#   YAW_SIGN      -> heading turn direction
FORWARD_SIGN = -1
ROLL_SIGN = 1
PITCH_SIGN = 1
YAW_SIGN = 1

roll_deg = 0.0
pitch_deg = 0.0
yaw_deg = 0.0
magyaw_deg = 0.0
packets = 0
last_line = ''

ser = serial.Serial()
ser.port = PORT
ser.baudrate = BAUD
ser.timeout = 1
ser.dtr = False
ser.rts = False
ser.open()
ser.reset_input_buffer()


def _read_serial():
    global roll_deg, pitch_deg, yaw_deg, magyaw_deg, packets, last_line
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            last_line = line

            if line.startswith('Gyro bias') or line.startswith('bias_') or line.startswith('time_ms') or line.startswith('IMU FAIL'):
                continue

            parts = line.split(',')
            if len(parts) < 4:
                continue

            _time_ms = float(parts[0])
            roll_deg = float(parts[1])
            pitch_deg = float(parts[2])
            yaw_deg = float(parts[3])
            if len(parts) >= 5:
                magyaw_deg = float(parts[4])
            packets += 1
        except Exception:
            pass


threading.Thread(target=_read_serial, daemon=True).start()

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.patch.set_facecolor('#1e1e1e')
ax.set_facecolor('#1e1e1e')

title = fig.suptitle('Waiting for CSV serial data...', color='white', fontsize=14)

ax.set_xlim(-1.4, 1.4)
ax.set_ylim(-1.4, 1.4)
ax.set_zlim(-1.2, 1.2)
ax.set_xlabel('X forward', color='lightgray')
ax.set_ylabel('Y right', color='lightgray')
ax.set_zlabel('Z up', color='lightgray')
ax.tick_params(colors='gray')
ax.set_box_aspect((1.4, 1.4, 1.0))

for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
    pane.fill = False
    pane.set_edgecolor('gray')

ax.view_init(elev=22, azim=-55)

# Fixed world axes
ax.quiver(0, 0, 0, 1.0, 0, 0, color='#aa3333', linewidth=1.5, arrow_length_ratio=0.08)
ax.quiver(0, 0, 0, 0, 1.0, 0, color='#33aa33', linewidth=1.5, arrow_length_ratio=0.08)
ax.quiver(0, 0, 0, 0, 0, 1.0, color='#3388ff', linewidth=1.5, arrow_length_ratio=0.08)

theta = np.linspace(0, 2 * np.pi, 150)
ax.plot(np.cos(theta), np.sin(theta), 0, color='#444444', linewidth=0.8, linestyle='--')

_dynamic_artists = []


def rotation_matrix(roll_r, pitch_r, yaw_r):
    cr, sr = np.cos(roll_r), np.sin(roll_r)
    cp, sp = np.cos(pitch_r), np.sin(pitch_r)
    cy, sy = np.cos(yaw_r), np.sin(yaw_r)

    rx = np.array([
        [1, 0, 0],
        [0, cr, -sr],
        [0, sr, cr]
    ])

    ry = np.array([
        [cp, 0, sp],
        [0, 1, 0],
        [-sp, 0, cp]
    ])

    rz = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1]
    ])

    return rz @ ry @ rx


def board_vertices(roll_r, pitch_r, yaw_r):
    length, width, t = 1.6, 1.0, 0.08
    corners = np.array([
        [-length / 2, -width / 2, -t / 2],
        [length / 2, -width / 2, -t / 2],
        [length / 2, width / 2, -t / 2],
        [-length / 2, width / 2, -t / 2],
        [-length / 2, -width / 2, t / 2],
        [length / 2, -width / 2, t / 2],
        [length / 2, width / 2, t / 2],
        [-length / 2, width / 2, t / 2],
    ], dtype=float)

    corners[:, 0] *= FORWARD_SIGN  # flip board nose/tail for display

    r = rotation_matrix(roll_r, pitch_r, yaw_r)
    return (r @ corners.T).T, r


def make_faces(v):
    return [
        [v[0], v[1], v[2], v[3]],
        [v[4], v[5], v[6], v[7]],
        [v[0], v[1], v[5], v[4]],
        [v[1], v[2], v[6], v[5]],
        [v[2], v[3], v[7], v[6]],
        [v[3], v[0], v[4], v[7]],
    ]


def update(_):
    global _dynamic_artists

    for artist in _dynamic_artists:
        try:
            artist.remove()
        except Exception:
            pass
    _dynamic_artists = []

    rr = np.radians(ROLL_SIGN * roll_deg)
    pr = np.radians(PITCH_SIGN * pitch_deg)
    yr = np.radians(YAW_SIGN * yaw_deg)

    v, r = board_vertices(rr, pr, yr)

    # Body axes
    x_axis = r @ np.array([0.9 * FORWARD_SIGN, 0, 0])
    y_axis = r @ np.array([0, 0.7, 0])
    z_axis = r @ np.array([0, 0, 0.7])

    qx = ax.quiver(0, 0, 0, *x_axis, color='red', linewidth=2, arrow_length_ratio=0.12)
    qy = ax.quiver(0, 0, 0, *y_axis, color='lime', linewidth=2, arrow_length_ratio=0.12)
    qz = ax.quiver(0, 0, 0, *z_axis, color='deepskyblue', linewidth=2, arrow_length_ratio=0.12)
    _dynamic_artists.extend([qx, qy, qz])

    # Board body
    faces = make_faces(v)
    face_colors = ['#7f8c8d', '#3498db', '#5dade2', '#f39c12', '#5dade2', '#c0392b']
    alphas = [0.35, 0.75, 0.55, 0.65, 0.55, 0.55]

    for f, c, a in zip(faces, face_colors, alphas):
        p = Poly3DCollection([f], facecolors=[c], edgecolors='white', linewidths=0.5)
        p.set_alpha(a)
        ax.add_collection3d(p)
        _dynamic_artists.append(p)

    # Nose arrow
    nose = r @ np.array([1.05 * FORWARD_SIGN, 0, 0])
    qf = ax.quiver(0, 0, 0, nose[0], nose[1], nose[2], color='yellow', linewidth=3, arrow_length_ratio=0.14)
    _dynamic_artists.append(qf)

    if packets == 0:
        title.set_text('Waiting for CSV serial data...')
    else:
        title.set_text(
            f'roll: {roll_deg:+.1f}°   pitch: {pitch_deg:+.1f}°   '
            f'yaw: {yaw_deg:+.1f}°   magYaw: {magyaw_deg:.1f}°   packets: {packets}'
        )


ani = animation.FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.tight_layout()

try:
    plt.show()
finally:
    ser.close()
