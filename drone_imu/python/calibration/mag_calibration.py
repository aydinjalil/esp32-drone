import argparse

import serial
import numpy as np

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import time

# Default is the USB flashing port; for an untethered Bluetooth sweep
# (mag_capture_bt.ino on battery) pass:  --port /dev/cu.DRONE_FC
parser = argparse.ArgumentParser(description="Live magnetometer capture + scatter plots")
parser.add_argument("--port", default="/dev/cu.usbserial-0001",
                    help="serial port (USB: /dev/cu.usbserial-0001, BT: /dev/cu.DRONE_FC)")
args = parser.parse_args()

serialPort = args.port
baud = 115200

to = .1
ser = serial.Serial(serialPort, baud, timeout=to)

log_file = open("mag_log.csv", "w")
log_file.write("mx,my,mz\n")
log_file.flush()

# Keep a large window so the full calibration sweep stays visible (a stall in
# the sensor used to scroll good points out after ~40 s at the old 2000 cap).
maxPoints = 20000

# Reads below this magnitude (uT) are treated as invalid sensor stalls and are
# neither plotted nor logged -- a cluster of zeros at the origin would corrupt
# the ellipsoid fit by dragging its centre toward zero.
MAG_MIN_VALID = 1.0
xRaw = np.empty(0)
yRaw = np.empty(0)
zRaw = np.empty(0)

app = QtWidgets.QApplication([])
mainWindow = QtWidgets.QWidget()
mainWindow.setWindowTitle("Magnetometer Calibration")
mainLayout = QtWidgets.QVBoxLayout()
mainWindow.setLayout(mainLayout)
plotLayout = QtWidgets.QGridLayout()
mainLayout.addLayout(plotLayout)

xyPlot = pg.PlotWidget(title="XY Plane")
xzPlot = pg.PlotWidget(title="XZ Plane")
yzPlot = pg.PlotWidget(title="YZ Plane")

for plot in [xyPlot, yzPlot, xzPlot]:
	plot.setAspectLocked(True)
	plot.showGrid(x=True, y=True)
	plot.setMinimumSize(300, 300)

plotLayout.addWidget(xyPlot, 0, 0)
plotLayout.addWidget(yzPlot, 0, 1)
plotLayout.addWidget(xzPlot, 1, 0)

xyScatter = pg.ScatterPlotItem(size=5)
yzScatter = pg.ScatterPlotItem(size=5)
xzScatter = pg.ScatterPlotItem(size=5)

xyPlot.addItem(xyScatter)
yzPlot.addItem(yzScatter)
xzPlot.addItem(xzScatter)

def updatePlot():
	global xRaw, yRaw, zRaw
	if ser.in_waiting > 0:
		try:
			line = ser.readline().decode('utf-8').strip()
			print(line)
			values = line.split(',')

			if len(values) == 3:
				x = float(values[0])
				y = float(values[1])
				z = float(values[2])

				# Drop stalled/invalid reads so they never reach the
				# plot or the calibration log.
				if (x * x + y * y + z * z) < (MAG_MIN_VALID * MAG_MIN_VALID):
					return

				xRaw = np.append(xRaw, x)[-maxPoints:]
				yRaw = np.append(yRaw, y)[-maxPoints:]
				zRaw = np.append(zRaw, z)[-maxPoints:]

				log_file.write(f"{x},{y},{z}\n")
				log_file.flush()

				xyScatter.setData(x=xRaw, y=yRaw, brush=pg.mkBrush(0, 0, 255, 120))
				yzScatter.setData(x=yRaw, y=zRaw, brush=pg.mkBrush(0, 255, 0, 120))
				xzScatter.setData(x=xRaw, y=zRaw, brush=pg.mkBrush(255, 0, 0, 120))

		except Exception as e:
			print("Parse Error: ", e)

plotTimer = QtCore.QTimer()
plotTimer.timeout.connect(updatePlot)
plotTimer.start(50)
mainWindow.show()

try:
	app.exec_()
finally:
	ser.close()
	log_file.close()