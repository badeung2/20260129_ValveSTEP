
import os
import shutil
import glob

TARGET_DIRS = [
    r"d:\Firmware\42_20260127_ValveBLDC\code\src\commlib",
    r"d:\Firmware\42_20260127_ValveBLDC\code\src\hal_lib"
]

for directory in TARGET_DIRS:
    print(f"Cleaning up backups in: {directory}")
    backup_dir = os.path.join(directory, "backup")
    
    if not os.path.exists(backup_dir):
        os.makedirs(backup_dir)
        print(f"Created backup directory: {backup_dir}")
        
    bak_files = glob.glob(os.path.join(directory, "*.bak"))
    
    for bak_file in bak_files:
        try:
            filename = os.path.basename(bak_file)
            dest_path = os.path.join(backup_dir, filename)
            # 덮어쓰기 이동
            if os.path.exists(dest_path):
                os.remove(dest_path)
            shutil.move(bak_file, dest_path)
            # print(f"Moved: {filename}")
        except Exception as e:
            print(f"Error moving {bak_file}: {e}")

    print(f"Moved {len(bak_files)} backup files.")

print("All backup files cleaned up.")
