import pandas as pd
import numpy as np

# กำหนด Path ของไฟล์ input และ output
input_file = '/home/kanchai/Desktop/SHM/sim/imu.log'
output_file = '/home/kanchai/Desktop/SHM/sim/imu_dual_diff_summary.txt'

try:
    # 1. อ่านไฟล์ log
    df = pd.read_csv(input_file)
    
    # 2. คำนวณผลต่างและแปลงเป็นหน่วย ms (หารด้วย 1,000,000) ของทั้งสองคอลัมน์
    sensor_diffs_ms = df['ts_sensor_ns'].diff().dropna() / 1e6
    shm_diffs_ms = df['ts_shm_ns'].diff().dropna() / 1e6
    
    # 3. คำนวณสถิติของ ts_sensor_ns
    s_mean = sensor_diffs_ms.mean()
    s_max = sensor_diffs_ms.max()
    s_min = sensor_diffs_ms.min()
    s_jit = sensor_diffs_ms.diff().dropna()
    s_mean_jit = s_jit.mean() if len(s_jit) > 0 else 0
    s_std_jit = s_jit.std(ddof=0) if len(s_jit) > 0 else 0
    s_max_abs_jit = s_jit.abs().max() if len(s_jit) > 0 else 0
    
    # 4. คำนวณสถิติของ ts_shm_ns
    sh_mean = shm_diffs_ms.mean()
    sh_max = shm_diffs_ms.max()
    sh_min = shm_diffs_ms.min()
    sh_jit = shm_diffs_ms.diff().dropna()
    sh_mean_jit = sh_jit.mean() if len(sh_jit) > 0 else 0
    sh_std_jit = sh_jit.std(ddof=0) if len(sh_jit) > 0 else 0
    sh_max_abs_jit = sh_jit.abs().max() if len(sh_jit) > 0 else 0
    
    # 5. สร้างข้อความ Summary ของทั้งสองคอลัมน์
    summary_text = f"""========================================
Summary: ts_sensor_ns (unit: millisecond)
========================================
Mean interval   : {s_mean:.3f} ms
Max interval    : {s_max:.3f} ms
Min interval    : {s_min:.3f} ms
Mean jitter     : {s_mean_jit:.3f} ms
Std jitter      : {s_std_jit:.3f} ms
Max |jitter|    : {s_max_abs_jit:.3f} ms

========================================
Summary: ts_shm_ns (unit: millisecond)
========================================
Mean interval   : {sh_mean:.3f} ms
Max interval    : {sh_max:.3f} ms
Min interval    : {sh_min:.3f} ms
Mean jitter     : {sh_mean_jit:.3f} ms
Std jitter      : {sh_std_jit:.3f} ms
Max |jitter|    : {sh_max_abs_jit:.3f} ms

========================================
Individual Diffs (ms) - format: ts_sensor, ts_shm
========================================
"""

    # 6. บันทึกข้อมูลลงในไฟล์เดียวกัน
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(summary_text)
        
        # เขียนค่า diff รายแถวคู่กัน (ts_sensor_ns, ts_shm_ns)
        for s_val, sh_val in zip(sensor_diffs_ms, shm_diffs_ms):
            f.write(f"{s_val:.3f}, {sh_val:.3f}\n")
            
    print(f"บันทึกข้อมูลสำเร็จแล้วที่: {output_file}")

except Exception as e:
    print(f"เกิดข้อผิดพลาด: {e}")