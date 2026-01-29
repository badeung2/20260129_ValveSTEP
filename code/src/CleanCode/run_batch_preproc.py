
import os
import subprocess
import glob

# 설정
TARGET_DIR = r"D:\Firmware\43_20260129_ValveSTEP\code\src"
SCRIPT_PATH = r"D:\Firmware\43_20260129_ValveSTEP\code\src\CleanCode\resolve_preprocessor.py"

# 대상 파일 확장자
EXTENSIONS = ['*.c', '*.h']

files = []
for ext in EXTENSIONS:
    files.extend(glob.glob(os.path.join(TARGET_DIR, ext)))

print(f"총 {len(files)}개의 파일에 대해 전처리문 제거를 시작합니다...")

for file_path in files:
    print(f"Processing: {os.path.basename(file_path)}")
    try:
        subprocess.run(["python", SCRIPT_PATH, file_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error processing {file_path}: {e}")

print("모든 작업이 완료되었습니다.")
