import serial

# เปิดพอร์ต /dev/ttyAMA0 ตั้งค่า Baud rate ให้ตรงกับตัวส่ง
# กำหนด timeout=1 เพื่อไม่ให้โปรแกรมค้างหากไม่มีข้อมูลเข้ามา
ser = serial.Serial('/dev/ttyAMA0', baudrate=115200, timeout=1)

print("กำลังรอรับข้อมูลจากขา RX (GPIO 15)... [กด Ctrl+C เพื่อเลิกทำงาน]")

try:
    while True:
        # ตรวจสอบว่ามีข้อมูลค้างอยู่ในบัฟเฟอร์ตัวรับหรือไม่
        if ser.in_waiting > 0:
            # อ่านข้อมูลเข้ามาจนเจอตัวขึ้นบรรทัดใหม่ (\n)
            raw_data = ser.readline()

            try:
                # แปลงข้อมูล bytes เป็นตัวหนังสือ (string) และตัดช่องว่าง/ตัวขึ้นบรรทัดใหม่ออก
                decoded_data = raw_data.decode('utf-8').strip()
                print(f"Received: {decoded_data}")
            except UnicodeDecodeError:
                # กรณีข้อมูลที่ส่งมาไม่ใช่ตัวหนังสือทั่วไป (เช่น ข้อมูลดิบจากเซนเซอร์)
                print(f"Received (Raw Bytes): {raw_data}")

except KeyboardInterrupt:
    print("\nหยุดการทำงาน")
finally:
    ser.close() # ปิดพอร์ตเมื่อจบโปรแกรม
