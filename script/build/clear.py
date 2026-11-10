#!/usr/bin/env python3
"""Stop local ROS processes in a selected domain; called by clear.sh."""
import argparse
import os
from pathlib import Path
import signal
import sys
import time

parser = argparse.ArgumentParser(description='Stop local ROS processes in one ROS_DOMAIN_ID')
parser.add_argument('--domain-id', type=int, default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
parser.add_argument('--list', action='store_true', help='only list matching local processes')
parser.add_argument('--yes', action='store_true', help='skip the interactive confirmation')
args = parser.parse_args()
if not 0 <= args.domain_id <= 232:
    parser.error('domain ID must be between 0 and 232')

# Exclude this command and all of its parent shells/launchers.
excluded = set()
pid = os.getpid()
while pid and pid not in excluded:
    excluded.add(pid)
    try:
        pid = int(Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[1])
    except (OSError, ValueError, IndexError):
        break


def inspect(pid):
    try:
        root = Path('/proc') / str(pid)
        if root.stat().st_uid != os.getuid():
            return None
        env = dict(item.split(b'=', 1) for item in (root / 'environ').read_bytes().split(b'\0') if b'=' in item)
        if int(env.get(b'ROS_DOMAIN_ID', b'0')) != args.domain_id:
            return None
        command = (root / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace').strip()
        if not command:
            return None
        maps = (root / 'maps').read_text(errors='replace')
        ros_process = ('/librcl.so' in maps or '/librcl.so.' in maps or
                       'ros2cli.daemon' in command or
                       ('/ros2' in command and (' launch ' in f' {command} ' or ' daemon ' in f' {command} ')))
        if not ros_process:
            return None
        start = (root / 'stat').read_text().rsplit(')', 1)[1].split()[19]
        return start, command
    except (OSError, ValueError, IndexError):
        return None

found = {}
for entry in Path('/proc').iterdir():
    if entry.name.isdigit() and int(entry.name) not in excluded:
        info = inspect(int(entry.name))
        if info:
            found[int(entry.name)] = info
print(f'本机当前用户、ROS_DOMAIN_ID={args.domain_id} 的可识别 ROS 进程：')
for pid, (_, command) in sorted(found.items()):
    print(f'  {pid}: {command}')
if not found:
    print('未发现可识别的进程。')
    sys.exit(0)
print('清理将中断控制和通信；它不会保证机械臂安全停机或失能。远程节点及无法识别的进程不在范围内。')
if args.list:
    sys.exit(0)
if not args.yes:
    if not sys.stdin.isatty() or input('确认停止以上进程？输入 yes 继续: ').strip() != 'yes':
        sys.exit('已取消。')
for sig in (signal.SIGINT, signal.SIGTERM):
    for pid, (start, _) in found.items():
        info = inspect(pid)
        if info and info[0] == start:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError as error:
                print(f'无法停止 {pid}: {error}', file=sys.stderr)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline and any(
            (info := inspect(pid)) and info[0] == start for pid, (start, _) in found.items()):
        time.sleep(0.1)
remaining = [pid for pid, (start, _) in found.items() if (info := inspect(pid)) and info[0] == start]
if remaining:
    sys.exit(f'这些进程尚未退出，请手动检查：{remaining}')
print('已请求的进程均已退出。')
