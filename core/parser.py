def parse_line(line):
    try:
        parts = line.split('|')
        print(f"num parts: {len(parts)}")

        alt = float(parts[0].split(':')[1].replace('m', '').strip())

        h3 = parts[1].split()
        print(f"h3: {h3}")
        hx = float(h3[1].split(':')[1])
        hy = float(h3[2].split(':')[1])
        hz = float(h3[3].split(':')[1])

        xl = parts[2].split()
        print(f"xl: {xl}")
        xl_x = float(xl[2].split(':')[1])
        xl_y = float(xl[3].split(':')[1])
        xl_z = float(xl[4].split(':')[1])

        gy = parts[3].split()
        print(f"gy: {gy}")
        gy_x = float(gy[1].split(':')[1])
        gy_y = float(gy[2].split(':')[1])
        gy_z = float(gy[3].split(':')[1])

        return alt, xl_x, xl_y, xl_z, gy_x, gy_y, gy_z, hx, hy, hz

    except Exception as e:
        print(f"Parse error: {e} | line: {line}")
        return None

import struct

RECEIVER_HEADER = b"\x00\x00\x00\x00"
RECEIVER_HEADER_LENGTH = len(RECEIVER_HEADER)
SYNC_WORD = 0xAA
PKT_TELEMETRY = 0x00
PKT_GPS = 0x01
PKT_CONTINUITY = 0x02
PKT_COMMAND = 0x03
GPS_RAW_PACKET_LENGTH = 37

CMD_ARM = 0x01
CMD_FIRE = 0x02
CMD_DISARM = 0x03
CMD_AUTH_BYTE = 0xBE

FLIGHT_STATES = {
    0: "IDLE",
    1: "PAD",
    2: "BOOST",
    3: "COAST",
    4: "APOGEE",
    5: "DROGUE",
    6: "PARAFOIL",
    7: "LAND",
}

def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def packet_type(data: bytes) -> int | None:
    """Return the application packet type from a raw receiver frame."""
    if len(data) < RECEIVER_HEADER_LENGTH + 2:
        return None
    if data[:RECEIVER_HEADER_LENGTH] != RECEIVER_HEADER:
        return None
    if data[RECEIVER_HEADER_LENGTH] != SYNC_WORD:
        return None
    return data[RECEIVER_HEADER_LENGTH + 1]


def _application_payload(data: bytes) -> bytes | None:
    if packet_type(data) is None:
        return None
    return data[RECEIVER_HEADER_LENGTH:]


def parse_telemetry(data: bytes) -> dict | None:
    payload = _application_payload(data)
    if payload is None or len(payload) < 58:
        return None
    if payload[1] != PKT_TELEMETRY:
        return None

    pkt_type = payload[1]
    seq = payload[2]

    (altitude, pressure, temperature,
     x_mg, y_mg, z_mg,
     x_mg_imu, y_mg_imu, z_mg_imu,
     x_gy, y_gy, z_gy,
     velocity) = struct.unpack('>fffffffffffff', payload[3:55])
    x_gy /= 1000.0
    y_gy /= 1000.0
    z_gy /= 1000.0

    flight_state = payload[55]

    crc_received = int.from_bytes(payload[56:58], "big")
    crc_calculated = crc16(payload[:56])
    if crc_received != crc_calculated:
        return None

    return {
        'packet_type': pkt_type,
        'sequence': seq,
        'altitude': altitude,
        'pressure': pressure,
        'temperature': temperature,
        'velocity': velocity,
        'h3lis':     {'x': x_mg,     'y': y_mg,     'z': z_mg},
        'imu_accel': {'x': x_mg_imu, 'y': y_mg_imu, 'z': z_mg_imu},
        'imu_gyro':  {'x': x_gy,     'y': y_gy,     'z': z_gy},
        'flight_state': flight_state,
    }


def parse_gps(data: bytes) -> dict | None:
    """Decode a complete 37-byte GPS radio frame, including receiver header."""
    if len(data) != GPS_RAW_PACKET_LENGTH:
        return None
    if data[:RECEIVER_HEADER_LENGTH] != RECEIVER_HEADER:
        return None

    payload = data[RECEIVER_HEADER_LENGTH:]
    if payload[0] != SYNC_WORD or payload[1] != PKT_GPS:
        return None

    crc_received = int.from_bytes(data[35:37], "big")
    crc_calculated = crc16(data[4:35])
    if crc_received != crc_calculated:
        return None

    status = data[7]
    (
        latitude_raw,
        longitude_raw,
        altitude_mm,
        ground_speed_cms,
        utc_ms,
        course_cdeg,
        information_age_ms,
        flight_state,
    ) = struct.unpack(">iiiIIHHB", data[10:35])

    return {
        "packet_type": PKT_GPS,
        "sequence": data[6],
        "status_flags": status,
        "fix_valid": bool(status & 0x01),
        "gga_received": bool(status & 0x02),
        "rmc_active": bool(status & 0x04),
        "uart_error": bool(status & 0x08),
        "fix_quality": data[8],
        "satellites": data[9],
        "latitude": latitude_raw / 10_000_000.0,
        "longitude": longitude_raw / 10_000_000.0,
        "altitude_m": altitude_mm / 1000.0,
        "ground_speed_mps": ground_speed_cms / 100.0,
        "utc_ms": utc_ms,
        "course_deg": course_cdeg / 100.0,
        "information_age_ms": information_age_ms,
        "flight_state": flight_state,
        "flight_state_name": FLIGHT_STATES.get(flight_state, f"UNKNOWN {flight_state}"),
    }


def parse_continuity(data: bytes) -> dict | None:
    payload = _application_payload(data)
    if payload is None or len(payload) < 8:
        return None
    if payload[1] != PKT_CONTINUITY:
        return None

    seq = payload[2]
    main = payload[3]
    drogue = payload[4]
    aux = payload[5]

    crc_received = int.from_bytes(payload[6:8], "big")
    crc_calculated = crc16(payload[:6])
    if crc_received != crc_calculated:
        return None

    return {
        'sequence': seq,
        'main':   bool(main),
        'drogue': bool(drogue),
        'aux':    bool(aux),
    }


def build_command(cmd_id: int, channel: int) -> bytes:
    buff = bytearray(9)
    buff[0] = SYNC_WORD
    buff[1] = PKT_COMMAND
    buff[2] = 0x00          # sequence — not validated on Odin
    buff[3] = cmd_id
    buff[4] = channel
    buff[5] = 0x00          # duration unused
    buff[6] = CMD_AUTH_BYTE
    crc = crc16(bytes(buff[:7]))
    buff[7] = (crc >> 8) & 0xFF
    buff[8] = crc & 0xFF
    return bytes(buff)
