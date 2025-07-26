import sys
import signal
import argparse
import struct
import numpy as np
from datetime import datetime, UTC
from PyQt6 import QtWidgets, QtCore
import pyqtgraph as pg

STRUCT_FORMAT = "<Qffff"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)

def load_samples(filename):
    with open(filename, "rb") as f:
        data = f.read()

    total_size = len(data)
    num_records = total_size // STRUCT_SIZE
    remainder = total_size % STRUCT_SIZE

    if remainder != 0:
        print(f"[WARN] File size is not a multiple of {STRUCT_SIZE}. {remainder} bytes ignored.")

    samples = []
    for i in range(num_records):
        offset = i * STRUCT_SIZE
        chunk = data[offset:offset + STRUCT_SIZE]
        try:
            sample = struct.unpack(STRUCT_FORMAT, chunk)
            samples.append(sample)
        except struct.error as e:
            print(f"[ERROR] Could not decode sample {i}: {e}")
            continue

    if not samples:
        raise RuntimeError("No valid data found in file.")

    timestamps, x, y, z, temp = zip(*samples)
    return np.array(timestamps), np.array(x), np.array(y), np.array(z), np.array(temp)

def format_xticks(timestamps, max_ticks=10):
    ticks = []
    n = len(timestamps)
    if n <= max_ticks:
        indices = range(n)
    else:
        spacing = max(n // max_ticks, 1)
        indices = range(0, n, spacing)

    for i in indices:
        ts = timestamps[i]
        label = datetime.fromtimestamp(ts // 1000, UTC).strftime("%H:%M:%S")
        ticks.append((i, label))

    return ticks

class Visualizer(QtWidgets.QMainWindow):
    def __init__(self, timestamps, x, y, z, temp):
        super().__init__()
        self.setWindowTitle("Magnetometer + Temperature Visualizer")
        self.timestamps = timestamps
        self.x = x
        self.y = y
        self.z = z
        self.temp = temp

        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        central.setLayout(layout)
        self.setCentralWidget(central)

        font = pg.QtGui.QFont()
        font.setPointSize(8)

        self.mag_plot = pg.PlotWidget(name="mag")
        self.mag_plot.setLabel('left', 'Magnetic Field [uT]')
        self.mag_plot.setLabel('bottom', 'Time [hh:mm:ss], UTC')
        self.mag_plot.showGrid(x=True, y=True)
        self.mag_plot.addLegend()
        self.mag_plot.getAxis('bottom').setTickFont(font)
        layout.addWidget(self.mag_plot)

        self.temp_plot = pg.PlotWidget(name="temp")
        self.temp_plot.setLabel('left', 'Temperature [°C]')
        self.temp_plot.setLabel('bottom', 'Time [hh:mm:ss]')
        self.temp_plot.showGrid(x=True, y=True)
        self.temp_plot.getAxis('bottom').setTickFont(font)
        layout.addWidget(self.temp_plot)

        self.temp_plot.setXLink(self.mag_plot)

        self.add_toolbar()
        self.plot_data()

        self.vline = pg.InfiniteLine(angle=90, movable=False, pen=pg.mkPen('y', width=1))
        self.mag_plot.addItem(self.vline)
        self.label = pg.TextItem(anchor=(0.5, 0.5), border='w', fill=(0, 0, 0, 150))
        self.mag_plot.addItem(self.label)
        self.proxy = pg.SignalProxy(self.mag_plot.scene().sigMouseMoved, rateLimit=60, slot=self.mouse_moved)

    def plot_data(self):
        indices = np.arange(len(self.timestamps))
        self.mag_plot.plot(indices, self.x, pen=pg.mkPen('r'), name='X')
        self.mag_plot.plot(indices, self.y, pen=pg.mkPen('g'), name='Y')
        self.mag_plot.plot(indices, self.z, pen=pg.mkPen('b'), name='Z')
        self.temp_plot.plot(indices, self.temp, pen=pg.mkPen('#FFA500', width=2), name='Temp')

        ticks = [format_xticks(self.timestamps, max_ticks=10)]
        self.mag_plot.getAxis('bottom').setTicks(ticks)
        self.temp_plot.getAxis('bottom').setTicks(ticks)

    def add_toolbar(self):
        toolbar = QtWidgets.QToolBar()
        self.addToolBar(toolbar)
        reset_btn = QtWidgets.QPushButton("Reset View")
        reset_btn.clicked.connect(self.reset_view)
        toolbar.addWidget(reset_btn)

    def reset_view(self):
        self.mag_plot.enableAutoRange()
        self.temp_plot.enableAutoRange()

    def mouse_moved(self, evt):
        pos = evt[0]
        if self.mag_plot.sceneBoundingRect().contains(pos):
            mouse_point = self.mag_plot.plotItem.vb.mapSceneToView(pos)
            index = int(mouse_point.x())
            if 0 <= index < len(self.timestamps):
                self.vline.setPos(index)
                label_text = (
                    f"Index: {index}\n"
                    f"Time: {datetime.fromtimestamp(self.timestamps[index] // 1000, UTC).strftime('%H:%M:%S')}\n"
                    f"X: {self.x[index]:.2f} uT\n"
                    f"Y: {self.y[index]:.2f} uT\n"
                    f"Z: {self.z[index]:.2f} uT\n"
                    f"Temp: {self.temp[index]:.2f} °C"
                )
                y_avg = (self.x[index] + self.y[index] + self.z[index]) / 3
                self.label.setText(label_text)
                self.label.setPos(index, y_avg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, help="Path to .bin file")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, lambda *args: QtWidgets.QApplication.quit())

    try:
        timestamps, x, y, z, temp = load_samples(args.file)
    except Exception as e:
        print(f"[FATAL] Error loading file: {e}")
        sys.exit(1)

    app = QtWidgets.QApplication(sys.argv)
    viewer = Visualizer(timestamps, x, y, z, temp)
    viewer.resize(1000, 800)
    viewer.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()

