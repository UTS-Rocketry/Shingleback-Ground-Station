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