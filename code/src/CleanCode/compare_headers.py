import difflib
import sys

file1_path = r"D:\Firmware\43_20260129_ValveSTEP\code\src\AppSwitches.h"
file2_path = r"D:\Firmware\43_20260129_ValveSTEP\code\src\AppSwitches_OL.h"

def read_file(path):
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        return f.readlines()

try:
    lines1 = read_file(file1_path)
    lines2 = read_file(file2_path)

    # Use comprehensive diff
    diff = list(difflib.unified_diff(lines1, lines2, fromfile='AppSwitches.h', tofile='AppSwitches_OL.h', n=0))
    
    with open(r"D:\Firmware\43_20260129_ValveSTEP\code\src\CleanCode\diff_output.txt", "w", encoding='utf-8') as out_f:
        if not diff:
            out_f.write("Files are identical.")
        else:
            out_f.write(f"Found {len(diff)} differences:\n")
            for line in diff:
                out_f.write(line)

except Exception as e:
    with open(r"D:\Firmware\43_20260129_ValveSTEP\code\src\CleanCode\diff_output.txt", "w", encoding='utf-8') as out_f:
        out_f.write(f"Error: {e}")
