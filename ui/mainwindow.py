import sys
from PyQt5 import QtWidgets, QtCore
from collections import deque
import pyqtgraph as pg

#from core.dummy_worker import DummyWorker

from core.serial_worker import SerialWorker
from core.parser import parse_line


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Shingleback Ground Station")
        self.resize(1400, 800)

        n = 100
        self.alt  = deque([0]*n, maxlen=n)
        self.gy_x = deque([0]*n, maxlen=n)
        self.gy_y = deque([0]*n, maxlen=n)
        self.gy_z = deque([0]*n, maxlen=n)
        self.xl_x = deque([0]*n, maxlen=n)
        self.xl_y = deque([0]*n, maxlen=n)
        self.xl_z = deque([0]*n, maxlen=n)

        # --- Plots ---
        self.plot_widget = pg.GraphicsLayoutWidget()

        alt_plot = self.plot_widget.addPlot(title="Altitude (m)")
        self.alt_curve = alt_plot.plot(pen='y')
        self.plot_widget.nextRow()

        gy_plot = self.plot_widget.addPlot(title="Gyro (mdps)")
        self.gy_x_curve = gy_plot.plot(pen='r', name="X")
        self.gy_y_curve = gy_plot.plot(pen='g', name="Y")
        self.gy_z_curve = gy_plot.plot(pen='b', name="Z")
        self.plot_widget.nextRow()

        xl_plot = self.plot_widget.addPlot(title="IMU Accel (mg)")
        self.xl_x_curve = xl_plot.plot(pen='r', name="X")
        self.xl_y_curve = xl_plot.plot(pen='g', name="Y")
        self.xl_z_curve = xl_plot.plot(pen='b', name="Z")

        # --- Dashboard ---
        dashboard = QtWidgets.QWidget()
        dash_layout = QtWidgets.QVBoxLayout()
        dashboard.setLayout(dash_layout)
        dashboard.setFixedWidth(220)

        def make_label(title):
            # helper — creates a bold section title
            lbl = QtWidgets.QLabel(title)
            lbl.setStyleSheet("font-weight: bold; font-size: 13px; margin-top: 8px;")
            return lbl

        def make_value():
            # helper — creates a value readout label
            lbl = QtWidgets.QLabel("--")
            lbl.setStyleSheet("font-family: Courier; font-size: 13px;")
            return lbl

        # Status
        dash_layout.addWidget(make_label("Status"))
        self.status_val = QtWidgets.QLabel("IDLE")
        self.status_val.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: orange;"
        )
        dash_layout.addWidget(self.status_val)

        # Altitude
        dash_layout.addWidget(make_label("Altitude"))
        self.alt_val = make_value()
        dash_layout.addWidget(self.alt_val)

        # IMU Accel
        dash_layout.addWidget(make_label("IMU Accel (mg)"))
        self.xl_x_val = make_value()
        self.xl_y_val = make_value()
        self.xl_z_val = make_value()
        dash_layout.addWidget(self.xl_x_val)
        dash_layout.addWidget(self.xl_y_val)
        dash_layout.addWidget(self.xl_z_val)

        # Gyro
        dash_layout.addWidget(make_label("Gyro (mdps)"))
        self.gy_x_val = make_value()
        self.gy_y_val = make_value()
        self.gy_z_val = make_value()
        dash_layout.addWidget(self.gy_x_val)
        dash_layout.addWidget(self.gy_y_val)
        dash_layout.addWidget(self.gy_z_val)

        # H3LIS
        dash_layout.addWidget(make_label("H3LIS (mg)"))
        self.hx_val = make_value()
        self.hy_val = make_value()
        self.hz_val = make_value()
        dash_layout.addWidget(self.hx_val)
        dash_layout.addWidget(self.hy_val)
        dash_layout.addWidget(self.hz_val)

        dash_layout.addStretch()  # pushes everything to the top

        # --- Terminal ---
        self.terminal = QtWidgets.QTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setFontFamily("Courier")

        # --- Layout ---
        # Splitter splits plots and terminal vertically
        v_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        v_splitter.addWidget(self.plot_widget)
        v_splitter.addWidget(self.terminal)
        v_splitter.setSizes([550, 150])

        # Horizontal layout — plots+terminal on left, dashboard on right
        h_layout = QtWidgets.QHBoxLayout()
        h_layout.addWidget(v_splitter)
        h_layout.addWidget(dashboard)

        central = QtWidgets.QWidget()
        central.setLayout(h_layout)
        self.setCentralWidget(central)

        # Worker
        self.worker = SerialWorker('/dev/ttyUSB0', 115200)
        #self.worker = DummyWorker()
        self.worker.data_received.connect(self.on_data)
        self.worker.start()

    def on_data(self, line):
        self.terminal.append(line)
        self.terminal.ensureCursorVisible()

        parsed = parse_line(line)
        if parsed is None:
            return

        alt, xl_x, xl_y, xl_z, gy_x, gy_y, gy_z, hx, hy, hz = parsed

        # Update buffers
        self.alt.append(alt)
        self.xl_x.append(xl_x)
        self.xl_y.append(xl_y)
        self.xl_z.append(xl_z)
        self.gy_x.append(gy_x)
        self.gy_y.append(gy_y)
        self.gy_z.append(gy_z)

        # Update plots
        self.alt_curve.setData(list(self.alt))
        self.gy_x_curve.setData(list(self.gy_x))
        self.gy_y_curve.setData(list(self.gy_y))
        self.gy_z_curve.setData(list(self.gy_z))
        self.xl_x_curve.setData(list(self.xl_x))
        self.xl_y_curve.setData(list(self.xl_y))
        self.xl_z_curve.setData(list(self.xl_z))

        # Update dashboard
        self.alt_val.setText(f"Alt: {alt:.2f} m")
        self.xl_x_val.setText(f"X: {xl_x:.1f}")
        self.xl_y_val.setText(f"Y: {xl_y:.1f}")
        self.xl_z_val.setText(f"Z: {xl_z:.1f}")
        self.gy_x_val.setText(f"X: {gy_x:.1f}")
        self.gy_y_val.setText(f"Y: {gy_y:.1f}")
        self.gy_z_val.setText(f"Z: {gy_z:.1f}")
        self.hx_val.setText(f"X: {hx:.1f}")
        self.hy_val.setText(f"Y: {hy:.1f}")
        self.hz_val.setText(f"Z: {hz:.1f}")