import re
import sys

def remove_comments(text):
    """
    Removes C/C++ style comments from text.
    - Handles // single line comments
    - Handles /* multi line comments */
    - Ignores comments inside strings "..."
    """
    def replacer(match):
        s = match.group(0)
        if s.startswith('/'):
            # It's a comment
            if s.startswith('/*'):
                # Multi-line comment
                # Check if it preserves formatting? 
                # Usually we replace with a single space to avoid merging tokens, 
                # unless it is a whole line.
                # But user wants "Delete".
                return " " 
            else:
                # Single line comment
                return ""
        else:
            # It's a string, return as is
            return s

    # Regex pattern:
    # Group 1: String literal "..." (handling escaped quotes)
    # Group 2: Single line comment // ... (until newline)
    # Group 3: Multi line comment /* ... */
    pattern = re.compile(
        r'("[^"\\]*(?:\\.[^"\\]*)*")|'  # Double-quoted String (Group 1)
        r"('[^'\\]*(?:\\.[^'\\]*)*')|"  # Single-quoted String (Group 2) - Char literal
        r'(//[^\n]*)|'                  # Single line (Group 3)
        r'(/\*[\s\S]*?\*/)',            # Multi line (Group 4)
        re.MULTILINE | re.DOTALL
    )

    # Remove comments
    content = pattern.sub(replacer, text)
    
    # Aggressive whitespace removal
    lines = content.splitlines()
    cleaned_lines = []
    for line in lines:
        stripped = line.strip()
        if stripped: # Only keep non-empty lines
            # Replace multiple spaces/tabs within line with a single space
            # But try to keep it very minimal. 
            # For preprocessor like #define A B, we need at least one space.
            line_cleaned = re.sub(r'[ \t]+', ' ', stripped)
            cleaned_lines.append(line_cleaned)
    
    return "\n".join(cleaned_lines)

def process_file(filepath):
    print(f"Processing {filepath} (removing comments and whitespace)...")
    with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()
    
    new_content = remove_comments(content)
    
    with open(filepath, 'w', encoding='utf-8', newline='\n') as f:
        f.write(new_content)
    print("Done.")

import os

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python remove_comments.py <filename_or_directory>")
    else:
        target = sys.argv[1]
        
        if os.path.isdir(target):
            # Process all .c and .h files in the directory (non-recursive)
            print(f"Scanning directory: {target} (Non-recursive)")
            files = [f for f in os.listdir(target) if f.endswith(('.c', '.h'))]
            
            if not files:
                print("No .c or .h files found in the directory.")
            
            for filename in files:
                filepath = os.path.join(target, filename)
                if os.path.isfile(filepath): # Ensure it's a file, not a dir ending in .c
                    process_file(filepath)
        else:
            # Process single file
            process_file(target)
