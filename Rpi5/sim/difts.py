#!/usr/bin/env python3
import sys
import csv

# bit layout: [63:48] counter | [47:34] value*100 | [33:0] ts_ms_of_day
COUNTER_SHIFT = 48
VALUE_SHIFT   = 34
COUNTER_MASK  = (1 << 16) - 1
VALUE_MASK    = (1 << 14) - 1
TS_MASK       = (1 << 34) - 1


def decode(packed: int):
    counter    = (packed >> COUNTER_SHIFT) & COUNTER_MASK
    value_bits = (packed >> VALUE_SHIFT) & VALUE_MASK
    ts_ms      = packed & TS_MASK

    value = value_bits / 100.0
    ts_sec = ts_ms / 1000.0
    h = int(ts_sec // 3600)
    m = int((ts_sec % 3600) // 60)
    s = ts_sec % 60
    ts_str = f"{h:02d}:{m:02d}:{s:06.3f}"

    return counter, value, ts_str


def main(in_path: str, out_path: str):
    with open(in_path, newline="") as fin, open(out_path, "w", newline="") as fout:
        reader = csv.DictReader(fin)
        writer = csv.writer(fout)
        writer.writerow(["packed", "counter", "value", "time_of_day"])

        count = 0
        for row in reader:
            packed = int(row["packed"])
            counter, value, ts_str = decode(packed)
            writer.writerow([packed, counter, value, ts_str])
            count += 1

    print(f"decode แล้ว {count} แถว -> {out_path}")


if __name__ == "__main__":
    in_path  = sys.argv[1] if len(sys.argv) > 1 else "reader_log.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "reader_log_decoded.csv"
    main(in_path, out_path)