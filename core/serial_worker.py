import serial
from PyQt5 import QtCore

class SerialWorker(QtCore.QThread):
    data_received = QtCore.pyqtSignal(str)

    def __init__(self, port, baud=115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
            while self.running:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.data_received.emit(line)
        except Exception as e:
            print(f"Serial error: {e}")

    def stop(self):
        self.running = False
        self.quit()