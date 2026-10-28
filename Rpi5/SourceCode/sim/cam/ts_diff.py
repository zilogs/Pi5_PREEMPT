import os
import pandas as pd
import numpy as np

# กำหนดพาธไฟล์อินพุตและไฟล์เอาต์พุต
input_file = '/home/kanchai/Desktop/SHM/sim/Log/process_timing.txt'
output_file = '/home/kanchai/Desktop/SHM/sim/Log/process_duration_summary.txt'

def process_timestamps():
    if not os.path.exists(input_file):
        print(f"Error: ไม่พบไฟล์อินพุตที่ {input_file}")
        return

    # คอลัมน์ที่บันทึกมาจาก Go (camcap.go)
    column_names = [
        'target_deadline', 't_wait_done', 't_pub_start', 't_pub_end',
        't_read_start', 't_read_end', 't_enc_start', 't_enc_end'
    ]

    try:
        df = pd.read_csv(input_file, sep=r'\s+', comment='#', names=column_names)
    except Exception as e:
        print(f"Error ในการอ่านไฟล์: {e}")
        return

    if df.empty:
        print("Error: ไฟล์ไม่มีข้อมูล")
        return

    df_calc = pd.DataFrame()
    df_calc['FRAME'] = range(1, len(df) + 1)

    # ----------------------------------------------------------------------------------
    # 1. การคำนวณระยะเวลา
    # ----------------------------------------------------------------------------------
    # 1.1 ความคลาดเคลื่อนเวลาตื่นนอนของโปรแกรม (Timer Wakeup Delay)
    df_calc['Wakeup_Delay(us)'] = (df['t_wait_done'] - df['target_deadline']) / 1e3

    # 1.2 เวลาเขียนภาพลง Shared Memory
    df_calc['SHM_Write_Time(us)'] = (df['t_pub_end'] - df['t_pub_start']) / 1e3

    # 1.3 เวลาว่างก่อนเริ่มอ่านภาพจากกล้อง (Preparation Overhead)
    df_calc['Pre_Read_Delay(us)'] = (df['t_read_start'] - df['t_pub_end']) / 1e3

    # 1.4 เวลาดึง/อ่านภาพจากกล้อง
    df_calc['Camera_Read_Time(ms)'] = (df['t_read_end'] - df['t_read_start']) / 1e6

    # 1.5 เวลาว่างก่อนเริ่ม แปลงไฟล์ภาพ (Pre-Encode Overhead)
    df_calc['Pre_Enc_Delay(us)'] = (df['t_enc_start'] - df['t_read_end']) / 1e3

    # 1.6 เวลาบีบอัด/แปลงภาพเป็น PNG
    df_calc['PNG_Encode_Time(ms)'] = (df['t_enc_end'] - df['t_enc_start']) / 1e6

    # 1.7 รวมเวลาทำงานจริงของ CPU ในรอบนั้น (Active CPU Processing Time)
    df_calc['Total_CPU_Work_Time(ms)'] = (
        (df['t_pub_end'] - df['t_pub_start']) +
        (df['t_read_end'] - df['t_read_start']) +
        (df['t_enc_end'] - df['t_enc_start'])
    ) / 1e6

    # 1.8 เวลารวมตั้งแต่เริ่มอ่านภาพจนพร้อมใช้งานจริง (Total Frame Latency)
    df_calc['Total_Frame_Latency(ms)'] = (df['t_pub_end'] - df['t_read_start'].shift(1)) / 1e6

    # 1.9 ระยะเวลาต่อรอบ/ความถี่เฟรมจริง (Cycle Time / Period)
    df_calc['Frame_Cycle_Time(ms)'] = df['t_pub_start'].diff() / 1e6

    # คำนวณค่าสถิติ
    avg = df_calc.mean()
    std = df_calc.std()
    max_val = df_calc.max()
    min_val = df_calc.min()

    # ----------------------------------------------------------------------------------
    # 2. บันทึกรายงานสรุป
    # ----------------------------------------------------------------------------------
    os.makedirs(os.path.dirname(output_file), exist_ok=True)

    # ตัวช่วย: วาดแถบ (bar) แสดงขนาดสัมพัทธ์เทียบกับค่าสูงสุด
    def bar(value, max_value, width=24):
        if value is None or max_value is None or max_value <= 0:
            return ""
        value = max(0.0, float(value))
        filled = int(round((value / max_value) * width))
        return "█" * filled + "░" * (width - filled)

    # รายการเมตริก: (ชื่อแสดง, คอลัมน์ข้อมูล)
    metrics = [
        ("Timer Jitter (us)",        "Wakeup_Delay(us)"),
        ("SHM Write (us)",           "SHM_Write_Time(us)"),
        ("Pre-Read Gap (us)",        "Pre_Read_Delay(us)"),
        ("Camera Read (ms)",         "Camera_Read_Time(ms)"),
        ("Pre-Enc Gap (us)",         "Pre_Enc_Delay(us)"),
        ("PNG Encode (ms)",          "PNG_Encode_Time(ms)"),
        ("CPU Work Time (ms)",       "Total_CPU_Work_Time(ms)"),
        ("Total Frame Latency (ms)", "Total_Frame_Latency(ms)"),
        ("Frame Cycle Time (ms)",    "Frame_Cycle_Time(ms)"),
    ]

    with open(output_file, 'w', encoding='utf-8') as f:
        rule = "═" * 92
        f.write(rule + "\n")
        f.write("  📷  PIPELINE PERFORMANCE SUMMARY\n")
        f.write(rule + "\n")
        f.write(f"  Input  : {input_file}\n")
        f.write(f"  Frames : {len(df)}\n")
        f.write(rule + "\n\n")

        # ---- ตารางสรุปเมตริก (Avg / Min / Max / Std + แถบแนวโน้ม) ----
        f.write(f"  {'Metric':<24}{'Avg':>12}{'Min':>12}{'Max':>12}{'Std':>12}   Trend\n")
        f.write("  " + "─" * 90 + "\n")
        for name, col in metrics:
            a, mn, mx, sd = avg[col], min_val[col], max_val[col], std[col]
            f.write(f"  {name:<24}{a:>12.3f}{mn:>12.3f}{mx:>12.3f}{sd:>12.3f}   {bar(a, mx)}\n")
        f.write("  " + "─" * 90 + "\n\n")

        # ---- ตารางรายเฟรม ----
        f.write("  PER-FRAME BREAKDOWN\n")
        f.write("  " + "─" * 90 + "\n")
        f.write(
            f"  {'#':>5} | {'Wake(us)':>10} | {'SHM(us)':>10} | {'PreRd(us)':>10} | "
            f"{'Cam(ms)':>10} | {'PreEn(us)':>10} | {'PNG(ms)':>10} | {'CPU(ms)':>10} | "
            f"{'Lat(ms)':>10} | {'Cyc(ms)':>10}\n"
        )
        f.write("  " + "─" * 90 + "\n")

        for _, row in df_calc.iterrows():
            lat = row['Total_Frame_Latency(ms)']
            cyc = row['Frame_Cycle_Time(ms)']
            lat_s = f"{lat:10.3f}" if pd.notna(lat) else f"{'N/A':>10}"
            cyc_s = f"{cyc:10.3f}" if pd.notna(cyc) else f"{'N/A':>10}"
            f.write(
                f"  {int(row['FRAME']):5d} | "
                f"{row['Wakeup_Delay(us)']:10.3f} | "
                f"{row['SHM_Write_Time(us)']:10.3f} | "
                f"{row['Pre_Read_Delay(us)']:10.3f} | "
                f"{row['Camera_Read_Time(ms)']:10.3f} | "
                f"{row['Pre_Enc_Delay(us)']:10.3f} | "
                f"{row['PNG_Encode_Time(ms)']:10.3f} | "
                f"{row['Total_CPU_Work_Time(ms)']:10.3f} | "
                f"{lat_s} | "
                f"{cyc_s}\n"
            )
        f.write("  " + "─" * 90 + "\n")

    print(f"บันทึกผลการคำนวณเรียบร้อยแล้วที่: {output_file}")

if __name__ == '__main__':
    process_timestamps()