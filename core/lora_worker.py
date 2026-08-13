import queue
import busio
import board
import digitalio
import adafruit_rfm9x
from PyQt5 import QtCore

class LoRaWorker(QtCore.QThread):
    RX_TIMEOUT_SECONDS = 0.02
    FREQUENCY_MHZ = 915.0
    SPREADING_FACTOR = 7
    SIGNAL_BANDWIDTH_HZ = 125000
    CODING_RATE_DENOMINATOR = 5
    PREAMBLE_LENGTH = 8
    LORA_SYNC_WORD = 0x12  # Fixed LoRa-mode default in adafruit_rfm9x.
    RADIOHEAD_HEADER = (0xFF, 0xFF, 0x00, 0x00)
    RADIOHEAD_HEADER_LENGTH = 4

    data_received = QtCore.pyqtSignal(bytes)
    error_occurred = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.running = False
        self.rfm9x = None
        self._send_queue = queue.Queue()

    def init_radio(self):
        try:
            spi = busio.SPI(board.SCLK, MOSI=board.MOSI, MISO=board.MISO)
            cs  = digitalio.DigitalInOut(board.D25)
            rst = digitalio.DigitalInOut(board.D27)
            self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, rst, self.FREQUENCY_MHZ)
            self.rfm9x.spreading_factor = self.SPREADING_FACTOR
            self.rfm9x.signal_bandwidth = self.SIGNAL_BANDWIDTH_HZ
            self.rfm9x.coding_rate = self.CODING_RATE_DENOMINATOR
            self.rfm9x.preamble_length = self.PREAMBLE_LENGTH
            self.rfm9x.enable_crc = True
            return True
        except Exception as e:
            self.error_occurred.emit(f"LoRa init failed: {str(e)}")
            return False

    def send(self, data: bytes):
        self._send_queue.put(data)  # just queue it, returns immediately

    def send_radiohead_packet(self, data: bytes):
        destination, node, identifier, flags = self.RADIOHEAD_HEADER
        try:
            self.rfm9x.send(
                data,
                keep_listening=True,
                destination=destination,
                node=node,
                identifier=identifier,
                flags=flags,
            )
        except TypeError:
            try:
                self.rfm9x.send(
                    data,
                    keep_listening=True,
                    tx_header=self.RADIOHEAD_HEADER,
                )
            except TypeError:
                self.rfm9x.send(data)

    def run(self):
        if not self.init_radio():
            return
        self.running = True
        while self.running:
            # send anything queued first
            try:
                data = self._send_queue.get_nowait()
                self.send_radiohead_packet(data)
            except queue.Empty:
                pass

            # then RX
            try:
                packet = self.rfm9x.receive(timeout=self.RX_TIMEOUT_SECONDS, with_header=True)
                if packet is not None:
                    # Keep the four receiver-routing bytes. Packet parsers validate
                    # and decode fields using the flight-computer's raw offsets.
                    self.data_received.emit(bytes(packet))
            except Exception as e:
                self.error_occurred.emit(f"LoRa RX error: {str(e)}")

    def stop(self):
        self.running = False
        self.wait()
