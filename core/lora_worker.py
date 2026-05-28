import busio
import board
import digitalio
import adafruit_rfm9x
from PyQt5 import QtCore

class LoRaWorker(QtCore.QThread):
    data_received = QtCore.pyqtSignal(bytes)
    error_occurred = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = False
        self.rfm9x = None

    def init_radio(self):
        try:
            spi = busio.SPI(board.SCLK, MOSI=board.MOSI, MISO=board.MISO)
            cs  = digitalio.DigitalInOut(board.D25)
            rst = digitalio.DigitalInOut(board.D27)
            self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, rst, 915.0)
            self.rfm9x.spreading_factor  = 7
            self.rfm9x.signal_bandwidth  = 125000
            self.rfm9x.coding_rate       = 5
            return True
        except Exception as e:
            self.error_occurred.emit(f"LoRa init failed: {str(e)}")
            return False

    def run(self):
        if not self.init_radio():
            return
        self.running = True
        while self.running:
            try:
                packet = self.rfm9x.receive(timeout=1.0, with_header=True)
                if packet is not None:
                    self.data_received.emit(bytes(packet[4:]))
            except Exception as e:
                self.error_occurred.emit(f"LoRa RX error: {str(e)}")

    def send(self, data: bytes):
        try:
            self.rfm9x.send(data)
        except Exception as e:
            self.error_occurred.emit(f"LoRa TX error: {str(e)}")

    def stop(self):
        self.running = False
        self.wait()