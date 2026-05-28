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

SYNC_WORD       = 0xAA
PKT_TELEMETRY   = 0x01
PKT_CONTINUITY  = 0x02
PKT_COMMAND     = 0x03
CMD_ARM         = 0x01
CMD_FIRE        = 0x02
CMD_DISARM      = 0x03
CMD_AUTH_BYTE   = 0xBE

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

def parse_telemetry(data: bytes) -> dict | None:
    if len(data) < 58:
        return None
    if data[0] != SYNC_WORD:
        return None
    if data[1] != PKT_TELEMETRY:
        return None

    pkt_type = data[1]
    seq = data[2]

    (altitude, pressure, temperature,
     x_mg, y_mg, z_mg,
     x_mg_imu, y_mg_imu, z_mg_imu,
     x_gy, y_gy, z_gy,
     velocity) = struct.unpack('>fffffffffffff', data[3:55])

    flight_state = data[55]

    crc_received   = (data[56] << 8) | data[57]
    crc_calculated = crc16(data[:56])
    print(f"CRC received: {crc_received:#06x} | CRC calculated: {crc_calculated:#06x}")
    if crc_received != crc_calculated:
        return None

    return {
        'packet_type': pkt_type,
        'sequence': seq,
        'altitude': altitude,
        'pressure': pressure,
        'temperature': temperature,
        'velocity': velocity,
        'accel':     {'x': x_mg,     'y': y_mg,     'z': z_mg},
        'imu_accel': {'x': x_mg_imu, 'y': y_mg_imu, 'z': z_mg_imu},
        'imu_gyro':  {'x': x_gy,     'y': y_gy,     'z': z_gy},
        'flight_state': flight_state,
    }

def parse_continuity(data: bytes) -> dict | None:
    if len(data) < 8:
        return None
    if data[0] != SYNC_WORD:
        return None
    if data[1] != PKT_CONTINUITY:
        return None

    seq    = data[2]
    main   = data[3]
    drogue = data[4]
    aux    = data[5]

    crc_received   = (data[6] << 8) | data[7]
    crc_calculated = crc16(data[:6])
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