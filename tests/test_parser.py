import struct
import unittest

from core.parser import (
    PKT_CONTINUITY,
    PKT_GPS,
    PKT_TELEMETRY,
    RECEIVER_HEADER,
    SYNC_WORD,
    crc16,
    packet_type,
    parse_continuity,
    parse_gps,
    parse_telemetry,
)


def gps_frame(*, corrupt_crc=False):
    payload = bytearray(33)
    payload[0] = SYNC_WORD
    payload[1] = PKT_GPS
    payload[2] = 255
    payload[3] = 0x07
    payload[4] = 2
    payload[5] = 11
    struct.pack_into(
        ">iiiIIHHB",
        payload,
        6,
        -338_654_321,
        1_512_098_765,
        123_456,
        4_321,
        45_296_789,
        27_055,
        65_535,
        3,
    )
    crc = crc16(payload[:31])
    payload[31:33] = crc.to_bytes(2, "big")
    if corrupt_crc:
        payload[32] ^= 0x01
    return RECEIVER_HEADER + bytes(payload)


class ParserTests(unittest.TestCase):
    def test_crc16_ccitt_xmodem_reference_vector(self):
        self.assertEqual(crc16(b"123456789"), 0x31C3)

    def test_parse_gps_uses_raw_offsets_and_units(self):
        frame = gps_frame()

        self.assertEqual(len(frame), 37)
        self.assertEqual(packet_type(frame), PKT_GPS)
        parsed = parse_gps(frame)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["sequence"], 255)
        self.assertTrue(parsed["fix_valid"])
        self.assertTrue(parsed["gga_received"])
        self.assertTrue(parsed["rmc_active"])
        self.assertFalse(parsed["uart_error"])
        self.assertEqual(parsed["fix_quality"], 2)
        self.assertEqual(parsed["satellites"], 11)
        self.assertAlmostEqual(parsed["latitude"], -33.8654321)
        self.assertAlmostEqual(parsed["longitude"], 151.2098765)
        self.assertAlmostEqual(parsed["altitude_m"], 123.456)
        self.assertAlmostEqual(parsed["ground_speed_mps"], 43.21)
        self.assertEqual(parsed["utc_ms"], 45_296_789)
        self.assertAlmostEqual(parsed["course_deg"], 270.55)
        self.assertEqual(parsed["information_age_ms"], 65_535)
        self.assertEqual(parsed["flight_state_name"], "COAST")

    def test_parse_gps_rejects_bad_crc_header_and_length(self):
        self.assertIsNone(parse_gps(gps_frame(corrupt_crc=True)))

        bad_header = bytearray(gps_frame())
        bad_header[0] = 0xFF
        self.assertIsNone(parse_gps(bytes(bad_header)))
        self.assertIsNone(parse_gps(gps_frame()[:-1]))

    def test_parse_telemetry_type_zero_with_receiver_header(self):
        payload = bytearray(58)
        payload[0] = SYNC_WORD
        payload[1] = PKT_TELEMETRY
        payload[2] = 17
        values = tuple(float(value) for value in range(1, 14))
        struct.pack_into(">fffffffffffff", payload, 3, *values)
        payload[55] = 2
        payload[56:58] = crc16(payload[:56]).to_bytes(2, "big")

        parsed = parse_telemetry(RECEIVER_HEADER + payload)

        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["packet_type"], 0x00)
        self.assertEqual(parsed["sequence"], 17)
        self.assertEqual(parsed["flight_state"], 2)
        self.assertEqual(parsed["altitude"], 1.0)
        self.assertEqual(parsed["velocity"], 13.0)
        self.assertEqual(parsed["imu_gyro"]["x"], 0.01)

    def test_parse_continuity_with_receiver_header(self):
        payload = bytearray((SYNC_WORD, PKT_CONTINUITY, 4, 1, 0, 1, 0, 0))
        payload[6:8] = crc16(payload[:6]).to_bytes(2, "big")

        parsed = parse_continuity(RECEIVER_HEADER + payload)

        self.assertEqual(
            parsed,
            {"sequence": 4, "main": True, "drogue": False, "aux": True},
        )


if __name__ == "__main__":
    unittest.main()
