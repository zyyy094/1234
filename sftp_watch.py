# SFTP 实时同步监听 - 轻量版
# 用法: python sftp_watch.py

import time, subprocess, os, sys
from pathlib import Path

# 解决 PowerShell 输出缓冲
sys.stdout.reconfigure(line_buffering=True)

LOCAL = Path(r"e:\fire_detect_project")
REMOTE = "user@192.168.43.231:/home/user/fire_detect_project"

# 只监听这些文件类型的变化
WATCH_EXTS = {".cpp", ".h", ".txt", ".py", ".cmake", ".onnx", ".jpg", ".md"}
SKIP_DIRS = {"build", ".git", ".vscode", ".idea", "__pycache__"}

def get_hash(filepath):
    try:
        return filepath.stat().st_size, filepath.stat().st_mtime
    except:
        return None

print("=" * 50)
print(" SFTP 实时同步监听")
print(f" 本地: {LOCAL}")
print(f" 远程: {REMOTE}")
print(" 编辑并保存文件，板子自动同步")
print(" Ctrl+C 退出")
print("=" * 50)

# 初始化文件状态
file_hashes = {}
for root, dirs, files in os.walk(LOCAL):
    dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
    for f in files:
        ext = os.path.splitext(f)[1].lower()
        if ext in WATCH_EXTS:
            fp = Path(root) / f
            rel = os.path.relpath(fp, LOCAL).replace("\\", "/")
            file_hashes[rel] = get_hash(fp)

print(f"监控中 ({len(file_hashes)} 个文件)...\n")

while True:
    time.sleep(2)
    for root, dirs, files in os.walk(LOCAL):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for f in files:
            ext = os.path.splitext(f)[1].lower()
            if ext not in WATCH_EXTS:
                continue
            fp = Path(root) / f
            rel = os.path.relpath(fp, LOCAL).replace("\\", "/")
            new_hash = get_hash(fp)
            
            if rel not in file_hashes or file_hashes.get(rel) != new_hash:
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] 检测变化 → {rel}")
                remote_file = f"{REMOTE}/{rel}"
                subprocess.run(f'scp "{fp}" {remote_file}', shell=True,
                              capture_output=True)
                print(f"        已同步!")
                file_hashes[rel] = new_hash
