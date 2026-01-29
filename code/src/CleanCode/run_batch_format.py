
import os
import subprocess
import glob

# 설정
TARGET_DIR = r"d:\Firmware\42_20260127_ValveBLDC\code\src\drivelib"
SCRIPT_PATH = r"d:\Firmware\42_20260127_ValveBLDC\code\src\format_code.py"

# 대상 파일 확장자 (헤더 파일 포함 여부는 선택적이나, 보통 포맷팅은 헤더도 함)
EXTENSIONS = ['*.c', '*.h']

files = []
for ext in EXTENSIONS:
    files.extend(glob.glob(os.path.join(TARGET_DIR, ext)))

print(f"총 {len(files)}개의 파일에 대해 포맷팅을 시작합니다...")

for file_path in files:
    print(f"Formatting: {os.path.basename(file_path)}")
    try:
        subprocess.run(["python", SCRIPT_PATH, file_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error formatting {file_path}: {e}")

print("모든 작업이 완료되었습니다.")
