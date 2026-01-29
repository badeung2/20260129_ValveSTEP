
import os
import subprocess
import glob

# 설정
TARGET_DIRS = [
    r"d:\Firmware\42_20260127_ValveBLDC\code\src\commlib",
    r"d:\Firmware\42_20260127_ValveBLDC\code\src\hal_lib"
]

# 스크립트 경로
BASE_DIR = r"d:\Firmware\42_20260127_ValveBLDC\code\src"
SCRIPT_REMOVE_COMMENTS = os.path.join(BASE_DIR, "remove_comments.py")
SCRIPT_RESOLVE_PREPROC = os.path.join(BASE_DIR, "resolve_preprocessor.py")
SCRIPT_FORMAT_CODE = os.path.join(BASE_DIR, "format_code.py")

EXTENSIONS = ['*.c', '*.h']

def run_step(script_path, file_path, step_name):
    try:
        # print(f"  [{step_name}] {os.path.basename(file_path)}")
        subprocess.run(["python", script_path, file_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"!!! Error in {step_name} for {file_path}: {e}")
        return False
    return True

def process_directory(directory):
    print(f"\nProcessing Directory: {directory}")
    files = []
    for ext in EXTENSIONS:
        files.extend(glob.glob(os.path.join(directory, ext)))
    
    # 정렬하여 순차 처리
    files.sort()
    
    total = len(files)
    print(f"Total files to process: {total}")

    for idx, file_path in enumerate(files):
        filename = os.path.basename(file_path)
        print(f"[{idx+1}/{total}] {filename} ... ", end='', flush=True)
        
        # 1. 주석 제거
        if not run_step(SCRIPT_REMOVE_COMMENTS, file_path, "Remove Comments"):
            print("FAILED at Comment Removal")
            continue
            
        # 2. 전처리문 제거
        if not run_step(SCRIPT_RESOLVE_PREPROC, file_path, "Resolve Preproc"):
            print("FAILED at Preproc Resolve")
            continue
            
        # 3. 코드 포맷팅
        if not run_step(SCRIPT_FORMAT_CODE, file_path, "Format Code"):
            print("FAILED at Formatting")
            continue
            
        print("Done.")

if __name__ == "__main__":
    print("=== Start Full Cleanup (commlib, hal_lib) ===")
    for d in TARGET_DIRS:
        process_directory(d)
    print("\n=== All Tasks Completed ===")
