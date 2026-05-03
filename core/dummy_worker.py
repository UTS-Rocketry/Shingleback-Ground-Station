import sys
import random
from PyQt5 import QtWidgets, QtCore

# --- Dummy data generator ---
# Simulates your STM32 printf output so you can develop without the board
# When ready for real hardware, replace this with the serial worker
class DummyWorker(QtCore.QThread):
    data_received = QtCore.pyqtSignal(str)  # signal that carries a string

    def run(self):
        # run() is what executes in the separate thread
        # QThread calls this automatically when you call .start()
        while True:
            alt   = round(random.uniform(-0.5, 10.0), 2)
            hx    = round(random.uniform(-300, 300), 1)
            hy    = round(random.uniform(-300, 300), 1)
            hz    = round(random.uniform(800, 1200), 1)
            xl_x  = round(random.uniform(-5, 5), 1)
            xl_y  = round(random.uniform(-5, 5), 1)
            xl_z  = round(random.uniform(990, 1010), 1)
            gy_x  = round(random.uniform(-200, 200), 1)
            gy_y  = round(random.uniform(-200, 200), 1)
            gy_z  = round(random.uniform(-200, 200), 1)

            line = (
                f"Alt: {alt}m | "
                f"H3LIS X:{hx} Y:{hy} Z:{hz} mg | "
                f"IMU XL X:{xl_x} Y:{xl_y} Z:{xl_z} mg | "
                f"GY X:{gy_x} Y:{gy_y} Z:{gy_z} mdps"
            )

            self.data_received.emit(line)   # emit sends the signal to whoever is listening
            QtCore.QThread.msleep(1000)     # wait 1 second between lines