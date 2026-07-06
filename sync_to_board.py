# 火焰检测项目 - 本地代码通过 SCP 同步到飞腾派
# 用法: python sync_to_board.py          （手动同步一次）
#       python sync_to_board.py --watch  （监听文件变化，每3秒自动同步）

import os
import sys
import time
import subprocess
from pathlib import Path

HOST = "192.168.43.231"
USER = "user"
LOCAL_PATH = Path(r"e:\fire_detect_project")
REMOTE_PATH = "/home/user/fire_detect_project"

# 排除的文件/目录
EXCLUDE = [
    "build", ".git", ".vscode", ".idea", "__pycache__",
    "*.exe", "*.out", "*.tgz", "*.tar.gz", "*.zip",
    "*.pyc", ".DS_Store", "Thumbs.db", "_sftp_test.txt",
    "sync_to_board.ps1", "sync_to_board.py",
]

def should_exclude(rel_path: str) -> bool:
    for part in rel_path.replace("\\", "/").split("/"):
        for pat in EXCLUDE:
            if pat.startswith("*") and part.endswith(pat[1:]):
                return True
            if pat == part:
                return True
    return False

def run_cmd(cmd, check=True):
    """运行 shell 命令"""
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if check and result.returncode != 0 and result.stderr:
        print(f"  警告: {result.stderr.strip()}")
    return result

def get_remote_mtime(rel_path: str) -> str:
    """获取远程文件修改时间"""
    result = subprocess.run(
        f'ssh {USER}@{HOST} "stat -c %Y /home/user/fire_detect_project/{rel_path} 2>/dev/null || echo 0"',
        shell=True, capture_output=True, text=True
    )
    return result.stdout.strip()

def do_sync():
    """扫描本地文件并增量推送到板子"""
    total = 0
    pushed = 0
    skipped = 0
    
    print("-" * 50)
    
    for root, dirs, files in os.walk(LOCAL_PATH):
        dirs[:] = [d for d in dirs if not should_exclude(
            os.path.relpath(os.path.join(root, d), LOCAL_PATH))]
        
        for fname in files:
            local_file = Path(root) / fname
            rel = os.path.relpath(local_file, LOCAL_PATH).replace("\\", "/")
            
            if should_exclude(rel):
                continue
            
            local_mtime = str(int(local_file.stat().st_mtime))
            total += 1
            
            # 比较远程文件时间戳（增量同步）
            remote_mtime = get_remote_mtime(rel)
            
            if remote_mtime == local_mtime:
                skipped += 1
                continue
            
            # 确保远程目录存在
            remote_dir = os.path.dirname(f"{REMOTE_PATH}/{rel}").replace("\\", "/")
            subprocess.run(
                f'ssh {USER}@{HOST} "mkdir -p {remote_dir}"',
                shell=True, capture_output=True
            )
            
            # 上传
            size = local_file.stat().st_size
            print(f"  ↑ {rel}  ({size} bytes)")
            scp_result = run_cmd(
                f'scp "{local_file}" {USER}@{HOST}:{REMOTE_PATH}/{rel}',
                check=False
            )
            
            # 同步修改时间
            subprocess.run(
                f'ssh {USER}@{HOST} "touch -d @{local_mtime} {REMOTE_PATH}/{rel}"',
                shell=True, capture_output=True
            )
            
            pushed += 1
    
    print(f"\n总计 {total} 文件 | 推送 {pushed} | 跳过 {skipped}")
    print("-" * 50)

def main():
    watch_mode = "--watch" in sys.argv or "-w" in sys.argv
    
    print("=" * 50)
    print(f"同步时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"本地: {LOCAL_PATH}")
    print(f"远程: {USER}@{HOST}:{REMOTE_PATH}")
    print("=" * 50)
    
    # 确保远程目录存在
    subprocess.run(
        f'ssh {USER}@{HOST} "mkdir -p {REMOTE_PATH}"',
        shell=True, capture_output=True
    )
    
    do_sync()
    print("同步完成！")
    
    if watch_mode:
        print(f"\n⏳ 监听模式 - 每 3 秒检查文件变化 (Ctrl+C 退出)")
        file_mtimes = {}
        try:
            while True:
                time.sleep(3)
                changed = False
                for root, dirs, files in os.walk(LOCAL_PATH):
                    dirs[:] = [d for d in dirs if not should_exclude(
                        os.path.relpath(os.path.join(root, d), LOCAL_PATH))]
                    for fname in files:
                        local_file = Path(root) / fname
                        rel = os.path.relpath(local_file, LOCAL_PATH).replace("\\", "/")
                        if should_exclude(rel):
                            continue
                        mtime = local_file.stat().st_mtime
                        if rel not in file_mtimes or file_mtimes[rel] != mtime:
                            changed = True
                            break
                    if changed:
                        break
                
                if changed:
                    print(f"\n[检测到变化] 同步中...")
                    do_sync()
                    print("继续监听...")
                
                # 重新记录
                file_mtimes = {}
                for root, dirs, files in os.walk(LOCAL_PATH):
                    dirs[:] = [d for d in dirs if not should_exclude(
                        os.path.relpath(os.path.join(root, d), LOCAL_PATH))]
                    for fname in files:
                        local_file = Path(root) / fname
                        rel = os.path.relpath(local_file, LOCAL_PATH).replace("\\", "/")
                        if should_exclude(rel):
                            continue
                        file_mtimes[rel] = local_file.stat().st_mtime
        except KeyboardInterrupt:
            print("\n监听已停止。")

if __name__ == "__main__":
    main()
