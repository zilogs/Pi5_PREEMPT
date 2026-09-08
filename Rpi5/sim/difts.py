#!/usr/bin/env python3
import sys
import csv

def main(in_path: str, out_path: str):
    with open(in_path, "r") as fin, open(out_path, "w", newline="") as fout:
        writer = csv.writer(fout)
        writer.writerow(["raw_bits", "decimal_value"])

        count = 0
        for line in fin:
            raw_str = line.strip()
            
            # ข้ามบรรทัดว่าง หรือบรรทัดที่เป็น Header
            if not raw_str or raw_str == "value":
                continue
            
            # ถ้าเป็นเลขฐานสอง 16 บิต ให้แปลงเป็นเลขฐานสิบ
            if all(c in '01' for c in raw_str) and len(raw_str) == 16:
                decimal_val = int(raw_str, 2)
                writer.writerow([raw_str, decimal_val])
                count += 1
            else:
                # เผื่อกรณีที่เป็นเลขฐานสิบอยู่แล้ว
                try:
                    decimal_val = int(raw_str)
                    writer.writerow([raw_str, decimal_val])
                    count += 1
                except ValueError:
                    pass

    print(f"แปลงและถอดรหัสข้อมูลเรียบร้อยแล้ว {count} แถว -> {out_path}")

if __name__ == "__main__":
    in_path  = sys.argv[1] if len(sys.argv) > 1 else "/home/kanchai/Desktop/SHM/sim/writer_log.log"
    out_path = sys.argv[2] if len(sys.argv) > 2 else "/home/kanchai/Desktop/SHM/sim/writer_decimal_log.csv"

    # in_path  = sys.argv[1] if len(sys.argv) > 1 else "/home/kanchai/Desktop/SHM/sim/reader_log.log"
    # out_path = sys.argv[2] if len(sys.argv) > 2 else "/home/kanchai/Desktop/SHM/sim/reader_decimal_log.csv"
    main(in_path, out_path)