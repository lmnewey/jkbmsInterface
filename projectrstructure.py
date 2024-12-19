# this is for contenating the whole project in to a single JSON file so i can feed it to claude AI and get help with the project 

import os
from pathlib import Path
import json
from typing import Dict, List

def scan_directory(directory: str, ignore_patterns: List[str] = None) -> Dict:
    """
    Scans a directory and returns its structure as a dictionary, including Python and HTML file contents.
    """
    if ignore_patterns is None:
        ignore_patterns = ['.venv', 'venv', '__pycache__', '.git']  # Add '.git' to ignore patterns
    
    directory_structure = {'name': os.path.basename(directory), 'type': 'directory', 'children': []}
    
    try:
        items = list(os.scandir(directory))
        
        for item in items:
            # Check if item should be ignored
            if any(pattern in item.path for pattern in ignore_patterns):
                continue
                
            if item.is_file():
                if item.name.endswith(('.h', '.c', '.cpp' )):  # Include both Python and HTML files
                    try:
                        with open(item.path, 'r', encoding='utf-8') as f:
                            content = f.read()
                            directory_structure['children'].append({
                                'name': item.name,
                                'type': 'file',
                                'size': os.path.getsize(item.path),
                                'extension': os.path.splitext(item.name)[1],
                                'content': content
                            })
                    except Exception:
                        pass  # Silently skip files that cannot be read
            elif item.is_dir():
                subdirectory = scan_directory(item.path, ignore_patterns)
                directory_structure['children'].append(subdirectory)  # Add all directories
    
    except PermissionError:
        pass  # Silently skip directories without permissions
    except Exception:
        pass  # Silently handle other unexpected errors
    
    # Sort children by name and type (directories first)
    directory_structure['children'].sort(key=lambda x: (x['type'] != 'directory', x['name'].lower()))
    
    return directory_structure


if __name__ == "__main__":
    current_dir = os.getcwd()
    print(f"Starting scan in: {current_dir}")  # Debug log
    
    structure = scan_directory(current_dir)
    
    output_file = "project_structure.json"
    with open(output_file, 'w', encoding='utf-8') as f:
        json.dump(structure, f, indent=2, ensure_ascii=False)
    
    print(f"\nScan complete. Structure saved to {output_file}")
    print("\nDirectory contents:")
    for item in os.listdir(current_dir):
        print(f"- {item}")
