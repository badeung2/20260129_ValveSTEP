import os
import re
import shutil
import sys

# =========================================================
# 설정: 미리 로드할 외부 헤더 파일 목록
CONFIG_HEADERS = [
    '../../verification/eclipse/pltf_defs.h',
    'ActDefines.h',
    'AppDefines.h',
    'AppSwitches.h',
    'ActParams.h',
    'AppBuild.h'
]

TARGET_FILE = 'ActADC.h'
IGNORED_MACROS = ['defined']
undefined_macros = set()

def parse_definitions(filepath, current_defs):
    if not os.path.exists(filepath):
        print(f"[경고] 설정 파일 {filepath}을(를) 찾을 수 없습니다.")
        return

    print(f"[정보] {filepath} 미리 로드 중...")
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"[오류] 파일 읽기 실패: {filepath} ({e})")
        return

    processed_lines = []
    i = 0
    while i < len(lines):
        line = lines[i].rstrip()
        while line.endswith('\\'):
            i += 1
            if i < len(lines):
                line = line[:-1] + lines[i].strip()
            else:
                break
        processed_lines.append(line)
        i += 1
        
    for line in processed_lines:
        line = line.strip()
        if not line.startswith('#'): continue
        content = line[1:].strip()
        
        if content.startswith('define'):
            remain = content[6:].strip()
            parts = remain.split(None, 1)
            macro = parts[0]
            val = parts[1] if len(parts) > 1 else '1'
            val = re.sub(r'//.*', '', val)
            val = re.sub(r'/\*.*?\*/', '', val)
            val = val.strip()
            if '(' in macro: continue 
            current_defs[macro] = clean_value(val)

def clean_value(val):
    if not isinstance(val, str): return val
    val = re.sub(r'\b(\d+)[UuLl]+\b', r'\1', val)
    val = re.sub(r'\b(0x[0-9A-Fa-f]+)[UuLl]+\b', r'\1', val)
    return val

def evaluate_condition(cond, defs):
    global undefined_macros
    
    # defined() 처리를 위한 함수
    def replace_defined(cond):
        def repl(match):
            macro = match.group(1)
            return '1' if macro in defs else '0'
        
        cond = re.sub(r'defined\s*\(\s*(\w+)\s*\)', repl, cond)
        cond = re.sub(r'defined\s+(\w+)', repl, cond)
        return cond

    def replace_macro(match):
        m = match.group(0)
        if m in defs:
            v = defs[m]
            if isinstance(v, str) and v in defs:
                v = defs[v]
            return str(v)
        elif re.match(r'^[A-Za-z_]\w*$', m):
            if m not in ['not', 'and', 'or', 'True', 'False', 'None']:
                # 미정의 매크로 발견 시 즉시 예외 발생 -> None 처리 유도
                undefined_macros.add(m)
                raise ValueError(m)
        return m

    # 매크로 반복 치환
    for _ in range(30):
        prev_cond = cond
        
        # 1. defined() 처리 (매크로 확장 후 나타날 수 있음)
        cond = replace_defined(cond)
        
        # 2. 연산자 변환 (매크로 확장 후 나타날 수 있음)
        cond = cond.replace('!=', '___NEQ___')
        cond = cond.replace('&&', ' and ').replace('||', ' or ').replace('!', ' not ')
        cond = cond.replace('___NEQ___', '!=')
        
        try:
            cond = re.sub(r'\b([A-Za-z_]\w*)\b', replace_macro, cond)
        except ValueError as e:
            return None # UNKNOWN
            
        cond = re.sub(r'\b(\d+)[UuLl]+\b', r'\1', cond)
        cond = re.sub(r'\b(0x[0-9A-Fa-f]+)[UuLl]+\b', r'\1', cond)
        
        if cond == prev_cond:
            break
            
    # 마지막으로 한번 더 연산자/defined 정리
    cond = replace_defined(cond)
    cond = cond.replace('!=', '___NEQ___')
    cond = cond.replace('&&', ' and ').replace('||', ' or ').replace('!', ' not ')
    cond = cond.replace('___NEQ___', '!=')

    try:
        return bool(eval(cond, {"__builtins__": None}, {}))
    except Exception as e:
        print(f"[DEBUG] Evaluation Error: cond='{cond}', error='{e}'")
        return None # 평가 불가

# Stack Status Enums
STATUS_SEEKING = 0 # 아직 True 블록을 찾지 못함 (앞선 블록들이 다 False)
STATUS_FOUND   = 1 # 현재 블록이 True임
STATUS_DONE    = 2 # 이미 앞선 블록에서 True가 나옴 (이후 블록은 다 스킵)

# Keep Mode Enums
KEEP_NONE = 0
KEEP_ALL  = 1 # 현재 블록(및 하위) 보존

def process_file_single_pass(filepath, initial_defs):
    backup_path = filepath + ".bak"
    if not os.path.exists(backup_path): # 백업이 없으면 생성 (있으면 유지)
        shutil.copy2(filepath, backup_path)

    print(f"[처리] {filepath} 처리 시작 (Keep Mode)...")
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
    except Exception as e:
        print(f"[오류] 파일 읽기 실패: {filepath} ({e})")
        return

    output_lines = []
    # Stack Item: [IsOutput, Status, KeepMode]
    # Root: 항상 출력, 이미 찾음(더 이상 조건 없음), Keep 없음
    stack = [[True, STATUS_DONE, KEEP_NONE]] 
    current_defs = initial_defs.copy()
    
    header_guard_macro = None
    if filepath.endswith('.h'):
        for line in lines:
            s = line.strip() 
            if s.startswith('#ifndef'):
                header_guard_macro = s[7:].strip().split()[0]
                break

    i = 0
    while i < len(lines):
        line = lines[i]
        full_line = line.rstrip()
        merged_count = 0
        temp_i = i
        while full_line.endswith('\\'):
            temp_i += 1
            if temp_i < len(lines):
                full_line = full_line[:-1] + lines[temp_i].strip()
                merged_count += 1
            else:
                break
        
        stripped = full_line.strip()
        
        if not stripped.startswith('#'):
            if stack[-1][0]: # IsOutput
                output_lines.append(line)
            i += 1 
            continue
            
        i += merged_count 
        content = stripped[1:].strip()
        
        is_parent_output = stack[-1][0]
        parent_keep_mode = stack[-1][2]
        
        # --- 지시어 처리 ---

        if content.startswith('define'):
            if is_parent_output:
                remain = content[6:].strip()
                parts = remain.split(None, 1)
                macro = parts[0]
                val = parts[1] if len(parts) > 1 else '1'
                val = re.sub(r'//.*', '', val)
                val = re.sub(r'/\*.*?\*/', '', val)
                val = val.strip()
                if '(' not in macro:
                    current_defs[macro] = clean_value(val)
                output_lines.append(full_line + '\n')

        elif content.startswith('undef'):
            if is_parent_output:
                macro = content[5:].strip().split()[0]
                if macro in current_defs:
                    del current_defs[macro]
                output_lines.append(full_line + '\n')

        elif content.startswith('ifdef'):
            macro = content[5:].strip().split()[0]
            # ifdef는 명확하므로 미정의 개념이 없음 (없으면 False일 뿐)
            # 하지만 Parent가 Keep이면 얘도 Keep
            if parent_keep_mode == KEEP_ALL:
                stack.append([True, STATUS_DONE, KEEP_ALL])
                output_lines.append(full_line + '\n')
            elif not is_parent_output:
                stack.append([False, STATUS_DONE, KEEP_NONE])
            else:
                res = (macro in current_defs) and (str(current_defs[macro]) != '0')
                if res:
                    stack.append([True, STATUS_FOUND, KEEP_NONE])
                else:
                    stack.append([False, STATUS_SEEKING, KEEP_NONE])

        elif content.startswith('ifndef'):
            macro = content[6:].strip().split()[0]
            is_guard = (header_guard_macro and macro == header_guard_macro and len(stack)==1)
            
            if is_guard:
                output_lines.append(full_line + '\n')
                stack.append([True, STATUS_FOUND, KEEP_NONE]) # 가드 내부는 평가해야 하므로 KEEP_NONE
            elif parent_keep_mode == KEEP_ALL:
                stack.append([True, STATUS_DONE, KEEP_ALL])
                output_lines.append(full_line + '\n')
            elif not is_parent_output:
                stack.append([False, STATUS_DONE, KEEP_NONE])
            else:
                res = (macro not in current_defs) or (str(current_defs[macro]) == '0')
                if res:
                    stack.append([True, STATUS_FOUND, KEEP_NONE])
                else:
                    stack.append([False, STATUS_SEEKING, KEEP_NONE])

        elif content.startswith('if'):
            cond = content[2:].strip()
            if parent_keep_mode == KEEP_ALL:
                stack.append([True, STATUS_DONE, KEEP_ALL])
                output_lines.append(full_line + '\n')
            elif not is_parent_output:
                stack.append([False, STATUS_DONE, KEEP_NONE])
            else:
                res = evaluate_condition(cond, current_defs)
                if res is None: # Unknown -> Keep
                    stack.append([True, STATUS_DONE, KEEP_ALL])
                    output_lines.append(full_line + '\n')
                elif res:
                    stack.append([True, STATUS_FOUND, KEEP_NONE])
                else:
                    stack.append([False, STATUS_SEEKING, KEEP_NONE])

        elif content.startswith('elif'):
            cond = content[4:].strip()
            # Pop prev context
            prev_state = stack.pop()
            prev_status = prev_state[1]
            prev_keep = prev_state[2]
            
            # Re-evaluate parent context
            is_parent_output = stack[-1][0]
            parent_keep_mode = stack[-1][2]
            
            if parent_keep_mode == KEEP_ALL:
                stack.append([True, STATUS_DONE, KEEP_ALL])
                output_lines.append(full_line + '\n')
            elif not is_parent_output:
                stack.append([False, STATUS_DONE, KEEP_NONE])
            else:
                # Parent is Active & Normal
                if prev_keep == KEEP_ALL:
                    # 형제가 Keep이었으므로 나도 Keep (문법 유지)
                    stack.append([True, STATUS_DONE, KEEP_ALL])
                    output_lines.append(full_line + '\n')
                elif prev_status == STATUS_FOUND or prev_status == STATUS_DONE:
                    # 이미 앞선 블록이 실행됨 -> 나는 Skip
                    stack.append([False, STATUS_DONE, KEEP_NONE])
                else: # STATUS_SEEKING
                    # 앞선 블록들이 다 False, 나를 평가
                    res = evaluate_condition(cond, current_defs)
                    if res is None:
                        # ★ 여기가 중요: 앞선 if들이 다 False여서 삭제됐는데
                        # 내가 Unknown이면, 나는 실행될 수도, 안될 수도 있음.
                        # 근데 앞의 #if가 사라졌으므로, 내가 #if 역할을 해야 함.
                        stack.append([True, STATUS_DONE, KEEP_ALL])
                        output_lines.append(f"#if {cond}\n") # elif -> if 변환 출력
                    elif res:
                        stack.append([True, STATUS_FOUND, KEEP_NONE])
                    else:
                        stack.append([False, STATUS_SEEKING, KEEP_NONE])

        elif content.startswith('else'):
            # Pop prev
            prev_state = stack.pop()
            prev_status = prev_state[1]
            prev_keep = prev_state[2]
            
            is_parent_output = stack[-1][0]
            parent_keep_mode = stack[-1][2]
            
            if parent_keep_mode == KEEP_ALL:
                 stack.append([True, STATUS_DONE, KEEP_ALL])
                 output_lines.append(full_line + '\n')
            elif not is_parent_output:
                 stack.append([False, STATUS_DONE, KEEP_NONE])
            else:
                 if prev_keep == KEEP_ALL:
                     stack.append([True, STATUS_DONE, KEEP_ALL])
                     output_lines.append(full_line + '\n')
                 elif prev_status == STATUS_FOUND or prev_status == STATUS_DONE:
                     stack.append([False, STATUS_DONE, KEEP_NONE])
                 else: # STATUS_SEEKING
                     # else니까 무조건 True
                     stack.append([True, STATUS_FOUND, KEEP_NONE])

        elif content.startswith('endif'):
            if len(stack) > 1:
                prev_state = stack.pop()
                # Keep 모드였거나, 헤더 가드를 닫는 경우(#endif) 출력
                if prev_state[2] == KEEP_ALL or (len(stack) == 1 and header_guard_macro):
                    output_lines.append(full_line + '\n')
            else:
                output_lines.append(full_line + '\n')

        else:
            if stack[-1][0]:
                output_lines.append(full_line + '\n')

        i += 1

    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            f.writelines(output_lines)
        print(f"[완료] {filepath} 처리됨")
    except Exception as e:
        print(f"[오류] 파일 쓰기 실패: {filepath} ({e})")


def main():
    print("=" * 40)
    print(" 전처리문 제거 스크립트 (Smart Keep Mode)")
    print("=" * 40)
    
    global undefined_macros
    undefined_macros = set()
    
    defs = {}
    defs['defined'] = 0 
    defs['TRUE'] = '1'
    defs['FALSE'] = '0'
    
    for h in CONFIG_HEADERS:
        parse_definitions(h, defs)
    print(f"[정보] 외부 정의 {len(defs)}개 로드됨.")

    target_files = []
    if len(sys.argv) > 1:
        # 커맨드 라인 인자가 있으면 그것들을 대상 파일로 사용
        target_files = sys.argv[1:]
    else:
        # 없으면 기본 TARGET_FILE 사용
        target_files = [TARGET_FILE]

    for fname in target_files:
        if os.path.exists(fname):
            process_file_single_pass(fname, defs)
        else:
            print(f"[오류] 대상 파일 {fname}이 없습니다.")

    report_path = 'undefined_macros_report.txt'
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write("미정의 전처리문 리포트\n")
        f.write("=======================\n")
        
        if not undefined_macros:
            f.write("특이사항 없음 (모든 조건이 명확하게 평가됨)\n")
        else:
             f.write("다음 매크로들이 정의되지 않아, 관련 조건문은 원본 그대로 유지되었습니다:\n")
             for m in sorted(undefined_macros):
                f.write(f"- {m}\n")
    
    print("=" * 40)
    print(f"[성공] 작업이 완료되었습니다. 결과 리포트: '{report_path}'")
    if undefined_macros:
        print(f"  -> 미정의 매크로 {len(undefined_macros)}개 발견. 해당 블록은 유지되었습니다.")

if __name__ == '__main__':
    main()
