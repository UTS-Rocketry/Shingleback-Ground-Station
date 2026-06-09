import queue
import busio
import board
import digitalio
import adafruit_rfm9x
from PyQt5 import QtCore

class LoRaWorker(QtCore.QThread):
    RX_TIMEOUT_SECONDS = 0.02
    PAYLOAD_SYNC_WORD = 0xAA
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
            self.rfm9x = adafruit_rfm9x.RFM9x(spi, cs, rst, 915.0)
            self.rfm9x.spreading_factor = 7
            self.rfm9x.signal_bandwidth = 125000
            self.rfm9x.coding_rate = 5
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

    def strip_radiohead_header(self, packet: bytes) -> bytes:
        if len(packet) >= 1 and packet[0] == self.PAYLOAD_SYNC_WORD:
            return bytes(packet)
        if (
            len(packet) > self.RADIOHEAD_HEADER_LENGTH
            and packet[self.RADIOHEAD_HEADER_LENGTH] == self.PAYLOAD_SYNC_WORD
        ):
            return bytes(packet[self.RADIOHEAD_HEADER_LENGTH:])
        return bytes(packet)

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
                    self.data_received.emit(self.strip_radiohead_header(packet))
            except Exception as e:
                self.error_occurred.emit(f"LoRa RX error: {str(e)}")

    def stop(self):
        self.running = False
        self.wait()
