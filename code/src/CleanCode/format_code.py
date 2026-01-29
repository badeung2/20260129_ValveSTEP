import os
import re
import sys

def format_file(filepath):
	try:
		with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
			lines = f.readlines()
	except Exception as e:
		print(f"Error reading {filepath}: {e}")
		return

	# ---------------------------------------------------------
	# 1. 전처리문 보호 (멀티라인 포함)
	# ---------------------------------------------------------
	protected_lines = []
	preproc_map = {}
	
	i = 0
	while i < len(lines):
		line = lines[i]
		stripped = line.lstrip()
		
		# 전처리문 시작 확인 (#으로 시작)
		if stripped.startswith('#'):
			# 멀티라인 전처리문인지 확인 (줄 끝에 \)
			full_preproc = line
			while line.strip().endswith('\\') and i + 1 < len(lines):
				i += 1
				line = lines[i]
				full_preproc += line
			
			# Placeholder 생성 (개행 없이 단순하게)
			placeholder = f"__PREPROC_PH_{len(preproc_map)}__"
			
			# 맵에 저장 (원본은 strip 하지 않음, 개행 포함)
			preproc_map[placeholder] = full_preproc.rstrip() 
			
			# 보호된 라인에는 placeholder만 추가 (나중에 한 줄로 취급)
			protected_lines.append(placeholder + "\n")
		else:
			protected_lines.append(line)
		i += 1
	
	content = "".join(protected_lines)

	# ---------------------------------------------------------
	# 2. 복합 연산자 보호 (중요: 공백 처리를 위해 미리 보호)
	# ---------------------------------------------------------
	# 사용자가 언급한 ==, !=, <=, >=, ->, <<, >> 등을 모두 보호
	# 순서 중요: 3글자(<<= 등) -> 2글자(== 등) -> 1글자
	ops_to_protect = [
		'->', '<<', '>>', '==', '!=', '<=', '>=', '&&', '||', '+=', '-=', '*=', '/='
	]
	
	op_map = {}
	def op_repl(match):
		s = match.group(0)
		ph = f"__OP_PH_{len(op_map)}__"
		op_map[ph] = s
		return ph
	
	# 정규식 패턴 생성 (이스케이프 필요)
	pattern_str = '|'.join(map(re.escape, ops_to_protect))
	content = re.sub(pattern_str, op_repl, content)

	# ---------------------------------------------------------
	# 3. 기본적인 공백 정리 및 키워드 표준화
	# ---------------------------------------------------------
	content = re.sub(r'\b(if|while|for|switch|return)\s+\(', r'\1(', content)
	content = re.sub(r'else\s+if', 'else if', content)
	
	# 중복 공백 제거
	content = re.sub(r'[ \t]{2,}', ' ', content)

	# ---------------------------------------------------------
	# 4. 멀티라인 헤더 통합 (if (...) \n { -> if (...) {)
	# ---------------------------------------------------------
	def merge_headers(text):
		keywords = ['if', 'while', 'for', 'else if']
		for kw in keywords:
			pattern = re.compile(rf'\b{re.escape(kw)}\s*\(')
			pos = 0
			while True:
				match = pattern.search(text, pos)
				if not match: break
				
				start = match.start()
				depth = 0
				end = -1
				for j in range(match.end() - 1, len(text)):
					if text[j] == '(': depth += 1
					elif text[j] == ')': depth -= 1
					if depth == 0:
						end = j
						break
				
				if end != -1:
					header_part = text[start:end+1]
					
					# 전처리문이 포함된 헤더는 병합하지 않음 (구조 깨짐 방지)
					if "__PREPROC_PH_" in header_part:
						pos = match.end()
						continue

					clean_header = re.sub(r'\s+', ' ', header_part).replace('( ', '(').replace(' )', ')')
					# 키워드 뒤 공백 제거 (if ( -> if()
					clean_header = clean_header.replace('if (', 'if(').replace('while (', 'while(').replace('for (', 'for(').replace('switch (', 'switch(')
					text = text[:start] + clean_header + text[end+1:]
					pos = start + len(clean_header)
				else:
					pos = match.end()
		return text
	
	content = merge_headers(content)

	# ---------------------------------------------------------
	# 5. 브레이스 스타일 정리 ({ -> \n{\n) (전처리문 보호된 상태)
	# ---------------------------------------------------------
	# 단순 치환 대신, 앞뒤 문맥 고려
	# 1. { 처리: 앞뒤 개행 추가
	content = re.sub(r'{\s*', '\n{\n', content)
	# 2. } 처리: 앞뒤 개행 추가
	content = re.sub(r'\s*}', '\n}\n', content)
	
	# 3. 깨진 패턴 복구 (}, ); )
	content = content.replace('\n}\n,', '\n},')
	content = content.replace('\n}\n;', '\n};')
	content = content.replace('\n}\n)', '\n})') # 매크로 함수 등 고려

	# ---------------------------------------------------------
	# 6. 보호된 복합 연산자 복구 (이제 구조 잡혔으니 연산자 복구)
	# ---------------------------------------------------------
	for ph, original in op_map.items():
		content = content.replace(ph, f" {original} ")

	# ---------------------------------------------------------
	# 7. 라인 단위 후처리 (들여쓰기, 리프 압축)
	# ---------------------------------------------------------
	raw_lines = [l.strip() for l in content.split('\n') if l.strip()]
	final_lines = []

	# 리프 압축 (Compaction) 로직
	i = 0
	while i < len(raw_lines):
		# else { } -> else{}
		if (i + 2 < len(raw_lines) and 
			raw_lines[i] == 'else' and raw_lines[i+1] == '{' and raw_lines[i+2] == '}'):
			final_lines.append(f"else{{}}")
			i += 3
		# if (...) { } -> if(...){}
		elif (i + 2 < len(raw_lines) and 
			  re.match(r'^(if|for|while|else if)\(.*\)$', raw_lines[i]) and 
			  raw_lines[i+1] == '{' and raw_lines[i+2] == '}'):
			final_lines.append(f"{raw_lines[i]}{{}}")
			i += 3
		# if (...) { stmt; } -> if(...){stmt;}
		elif (i + 3 < len(raw_lines) and 
			  re.match(r'^(if|for|while|else if)\(.*\)$', raw_lines[i]) and 
			  raw_lines[i+1] == '{' and 
			  ';' in raw_lines[i+2] and '{' not in raw_lines[i+2] and '}' not in raw_lines[i+2] and
			  # 중첩 브레이스 없는 경우만
			  raw_lines[i+3] == '}'):
			final_lines.append(f"{raw_lines[i]}{{{raw_lines[i+2]}}}")
			i += 4
		# else { stmt; } -> else{stmt;}
		elif (i + 3 < len(raw_lines) and 
			  raw_lines[i] == 'else' and 
			  raw_lines[i+1] == '{' and 
			  ';' in raw_lines[i+2] and '{' not in raw_lines[i+2] and '}' not in raw_lines[i+2] and
			  raw_lines[i+3] == '}'):
			final_lines.append(f"else{{{raw_lines[i+2]}}}")
			i += 4
		else:
			# 배열 초기화 등에서 {{ }} 패턴이 발생하면 좀 붙여주기
			# 예: { 가 나오고 다음줄이 { 면? (일단 패스, 들여쓰기만 잘 맞아도 됨)
			final_lines.append(raw_lines[i])
			i += 1

	# ---------------------------------------------------------
	# 8. 들여쓰기 적용 및 전처리문 복구
	# ---------------------------------------------------------
	output_lines = []
	indent = 0
	
	for line in final_lines:
		# 전처리문 복구 (완전 일치로 확인)
		if line in preproc_map:
			output_lines.append(preproc_map[line])
			continue
		
		# 혹시 모르니 부분 일치 매칭 (안전 장치)
		is_preproc = False
		for ph, orig in preproc_map.items():
			if ph in line:
				output_lines.append(line.replace(ph, orig))
				is_preproc = True
				break
		if is_preproc: continue

		# 닫는 브레이스로 시작하면, 현재 라인은 내어쓰기
		current_line_indent = indent
		if line.strip().startswith('}'):
			current_line_indent = max(0, indent - 1)
			
		output_lines.append(('\t' * current_line_indent) + line)
		
		# 다음 라인을 위한 인덴트 계산
		opens = line.count('{')
		closes = line.count('}')
		indent += (opens - closes)
		indent = max(0, indent) # 음수 방지
		
		# 함수 간 공백
		if line == '}' and indent == 0:
			output_lines.append("")

	# 최종 조합
	final_content = '\n'.join(output_lines)
	
	# 문법적 공백 정리 (include 뒤 등)
	final_content = re.sub(r'(#include[^\n]+)\n(#pragma|typedef|void|static|uint|int|extern)', r'\1\n\n\2', final_content)
	final_content = re.sub(r'\n{3,}', '\n\n', final_content)

	with open(filepath, 'w', encoding='utf-8') as f:
		f.write(final_content.strip() + '\n')

def main():
	target = sys.argv[1] if len(sys.argv) > 1 else os.getcwd()
	if os.path.isfile(target):
		print(f"Formatting file: {target}")
		format_file(target)
	elif os.path.isdir(target):
		files = [f for f in os.listdir(target) if f.endswith(('.c', '.h')) and f != "format_code.py"]
		for f in files:
			print(f"Formatting: {f}")
			format_file(os.path.join(target, f))
		print(f"Done. {len(files)} files processed.")

if __name__ == "__main__":
	main()
