import pandas as pd
import matplotlib.pyplot as plt

def generate_timing_and_analysis():
    # 1. โหลดข้อมูลจากไฟล์ Log ทั้งสองฝั่ง
    try:
        writer_df = pd.read_csv('writer_log.csv')
        reader_df = pd.read_csv('reader_log.csv')
    except FileNotFoundError as e:
        print(f"ไม่พบไฟล์ Log: {e}. กรุณารันโปรแกรม C++ เพื่อสร้างไฟล์ log ก่อนครับ")
        return

    print(f"โหลดข้อมูลสำเร็จ: Writer ({len(writer_df)} แถว), Reader ({len(reader_df)} แถว)")

    # ==========================================
    # ส่วนที่ 1: สร้าง PlantUML Timing Diagram
    # ==========================================
    n_samples = 15  # เลือกตัวอย่าง 15 บรรทัดแรกเพื่อให้อ่านง่าย
    w_subset = writer_df.head(n_samples)
    r_subset = reader_df.head(n_samples)

    puml_content = "@startuml\n"
    puml_content += "skinparam Monochrome true\n"
    puml_content += "title IPC Timing Diagram (Writer vs Reader)\n\n"

    puml_content += "robust \"Writer State\" as W\n"
    puml_content += "robust \"Reader State\" as R\n"
    puml_content += "concise \"Shared Memory (counter)\" as SHM\n\n"

    puml_content += "@0\n"
    puml_content += "W is Idle\n"
    puml_content += "R is Waiting\n"
    puml_content += "SHM is 0\n\n"

    t_start = w_subset['ts'].min()

    for idx, row in w_subset.iterrows():
        t_w_ms = int((row['ts'] - t_start) * 1000)
        counter = int(row['counter'])
        
        puml_content += f"@{t_w_ms}\n"
        puml_content += f"W is Writing (c={counter})\n"
        puml_content += f"SHM is {counter}\n"

    for idx, row in r_subset.iterrows():
        t_r_ms = int((row['ts_r'] - t_start) * 1000)
        counter = int(row['counter'])
        
        puml_content += f"@{t_r_ms}\n"
        puml_content += f"R is Reading (c={counter})\n"

    puml_content += "\n@enduml"

    with open("timing_diagram.puml", "w", encoding="utf-8") as f:
        f.write(puml_content)
    print("-> สร้างไฟล์ 'timing_diagram.puml' สำเร็จแล้ว!")

    # ==========================================
    # ส่วนที่ 2: สร้างกราฟวิเคราะห์ Latency ด้วย Matplotlib
    # ==========================================
    merged_df = pd.merge(writer_df, reader_df, on='counter', suffixes=('_w', '_r'))
    merged_df['latency_ms'] = (merged_df['ts_r'] - merged_df['ts']) * 1000
    merged_df['time_sec'] = merged_df['ts'] - t_start

    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(merged_df['time_sec'], merged_df['value_w'], label='Value Written', color='blue', alpha=0.7)
    plt.ylabel('Value')
    plt.title('Writer-Reader Data Flow and IPC Latency Timing Analysis')
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(merged_df['time_sec'], merged_df['latency_ms'], label='IPC Latency (ms)', color='red')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Latency (ms)')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.savefig('ipc_timing_analysis.png', dpi=300)
    plt.show()
    print("-> บันทึกกราฟรูปภาพ 'ipc_timing_analysis.png' สำเร็จแล้ว!")

if __name__ == '__main__':
    generate_timing_and_analysis()