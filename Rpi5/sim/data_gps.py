import os
import kagglehub

# ดาวน์โหลดชุดข้อมูล
path = kagglehub.dataset_download("arashnic/microsoft-geolife-gps-trajectory-dataset")

# ค้นหาไฟล์ .plt ไฟล์แรกโดยใช้ os.walk
full_plt_path = None
for dirpath, dirnames, filenames in os.walk(path):
    for filename in filenames:
        if filename.endswith('.plt'):
            full_plt_path = os.path.join(dirpath, filename)
            break 
    if full_plt_path:
        break 

if full_plt_path:
    print(f"แสดง 10 บรรทัดแรกของไฟล์: {full_plt_path}")
    with open(full_plt_path, 'r') as f:
        for i, line in enumerate(f):
            if i >= 10:
                break
            print(line.strip())
else:
    print(f"ไม่พบไฟล์ .plt ในชุดข้อมูลที่ดาวน์โหลด")