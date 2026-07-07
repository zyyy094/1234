#!/usr/bin/env python3
"""
fire_web_server.py - 消防通道检测系统 Web 服务器

功能:
  - 实时显示检测状态（监控/预警/占用/火灾）
  - 查看检测统计数据（FPS、照片数、运行时间）
  - 浏览/下载 captures/ 目录中的截图
  - 查看检测日志
  - 自动刷新（每 3 秒）

用法:
  python3 fire_web_server.py [port]
  默认端口: 8080

手机/电脑访问: http://<飞腾派IP>:8080
"""

import os
import json
import time
import glob
import subprocess
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from datetime import datetime

# ====================== 配置 ======================
PROJECT_DIR = '/home/user/fire_detect_project'
CAPTURES_DIR = os.path.join(PROJECT_DIR, 'captures')
STATUS_FILE = os.path.join(PROJECT_DIR, 'status.json')
LOG_FILE = os.path.join(PROJECT_DIR, 'fire_detect.log')
PORT = 8080

# ====================== 状态读取 ======================
def read_status():
    """读取 fire_detect 程序写入的状态文件"""
    default = {
        'state': 'UNKNOWN',
        'state_color': '#888',
        'fps': 0,
        'photo_count': 0,
        'flame_detected': False,
        'timer': 0,
        'countdown': 0,
        'timestamp': '',
        'uptime': 0,
    }
    try:
        if os.path.exists(STATUS_FILE):
            with open(STATUS_FILE, 'r') as f:
                data = json.load(f)
                default.update(data)
    except:
        pass
    return default

def get_captures():
    """获取截图列表"""
    files = []
    if os.path.exists(CAPTURES_DIR):
        for f in sorted(glob.glob(os.path.join(CAPTURES_DIR, '*.jpg')), reverse=True):
            stat = os.stat(f)
            files.append({
                'name': os.path.basename(f),
                'size': stat.st_size,
                'time': datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S'),
                'size_str': f'{stat.st_size/1024:.1f} KB'
            })
    return files

def get_log_tail(lines=50):
    """读取日志最后 N 行"""
    try:
        if os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'r') as f:
                all_lines = f.readlines()
                return ''.join(all_lines[-lines:])
    except:
        pass
    return '暂无日志'

# ====================== HTML 页面 ======================
HTML_PAGE = '''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>消防通道检测系统</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family:-apple-system,BlinkMacSystemFont,"Segoe UI","Noto Sans CJK SC",sans-serif;
       background:#0f0f12; color:#e8e8ec; min-height:100vh; }
.header { background:linear-gradient(135deg,#1a1a22,#252530); padding:20px; text-align:center;
          border-bottom:1px solid #333; }
.header h1 { font-size:22px; margin-bottom:4px; }
.header .sub { color:#888; font-size:13px; }
.container { max-width:900px; margin:0 auto; padding:16px; }
.card { background:#1c1c22; border:1px solid #2a2a32; border-radius:12px;
        padding:20px; margin-bottom:16px; }
.card-title { color:#7a7a85; font-size:12px; font-weight:bold; text-transform:uppercase;
              letter-spacing:1px; margin-bottom:12px; }
.status-badge { display:inline-flex; align-items:center; gap:8px; padding:8px 20px;
                border-radius:20px; font-size:18px; font-weight:bold; }
.status-dot { width:12px; height:12px; border-radius:50%; background:#fff; }
.grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(140px,1fr)); gap:12px; }
.metric { background:#252530; border-radius:8px; padding:16px; text-align:center; }
.metric .label { color:#7a7a85; font-size:11px; margin-bottom:4px; }
.metric .value { font-size:24px; font-weight:bold; }
.metric .unit { font-size:12px; color:#888; }
.captures-list { max-height:400px; overflow-y:auto; }
.capture-item { display:flex; align-items:center; justify-content:space-between;
                padding:10px; border-bottom:1px solid #2a2a32; }
.capture-item:hover { background:#252530; }
.capture-item img { width:48px; height:48px; object-fit:cover; border-radius:6px; }
.capture-info { flex:1; margin-left:12px; }
.capture-name { font-size:13px; }
.capture-meta { font-size:11px; color:#666; }
.capture-dl { background:#238636; color:#fff; border:none; padding:6px 14px;
              border-radius:6px; cursor:pointer; font-size:12px; text-decoration:none; }
.log-box { background:#0a0a0e; border-radius:8px; padding:12px; font-family:monospace;
           font-size:12px; max-height:300px; overflow-y:auto; white-space:pre-wrap;
           color:#9aca3a; }
.footer { text-align:center; color:#555; font-size:12px; padding:20px; }
@media(max-width:600px){ .header h1{font-size:18px} .metric .value{font-size:20px} }
</style>
</head>
<body>
<div class="header">
<h1>🔥 消防通道检测系统</h1>
<div class="sub">飞腾派 PE2204 · 实时监控面板 · <span id="time"></span></div>
</div>
<div class="container">

<!-- 状态卡片 -->
<div class="card">
<div class="card-title">当前状态</div>
<div id="status-badge" class="status-badge" style="background:#2ecc71">
<span class="status-dot"></span><span id="status-text">MONITORING</span>
</div>
<div id="flame-alert" style="display:none;margin-top:12px;color:#ff4444;font-weight:bold;">
⚠ 检测到火焰！蜂鸣器已启动
</div>
</div>

<!-- 数据指标 -->
<div class="card">
<div class="card-title">实时数据</div>
<div class="grid">
<div class="metric"><div class="label">FPS</div><div class="value" id="fps">0</div></div>
<div class="metric"><div class="label">照片数</div><div class="value" id="photos">0</div></div>
<div class="metric"><div class="label">计时器</div><div class="value" id="timer">0<span class="unit">s</span></div></div>
<div class="metric"><div class="label">倒计时</div><div class="value" id="countdown">--</div></div>
<div class="metric"><div class="label">运行时间</div><div class="value" id="uptime">0<span class="unit">min</span></div></div>
<div class="metric"><div class="label">更新时间</div><div class="value" id="update-time" style="font-size:14px">--</div></div>
</div>
</div>

<!-- 截图列表 -->
<div class="card">
<div class="card-title">检测截图 (<span id="cap-count">0</span>)</div>
<div id="captures" class="captures-list"></div>
</div>

<!-- 日志 -->
<div class="card">
<div class="card-title">系统日志</div>
<div id="log" class="log-box">加载中...</div>
</div>

</div>
<div class="footer">飞腾派异构双核消防检测系统 · 局域网访问 · 自动刷新3s</div>

<script>
const STATE_COLORS = {
  'MONITORING':'#2ecc71', 'WARNING':'#f39c12',
  'OCCUPIED':'#e74c3c', 'FIRE':'#ff1744', 'UNKNOWN':'#888'
};

async function refresh() {
  try {
    const res = await fetch('/api/status');
    const data = await res.json();

    // 状态
    const badge = document.getElementById('status-badge');
    const color = data.state_color || STATE_COLORS[data.state] || '#888';
    badge.style.background = color;
    document.getElementById('status-text').textContent = data.state;

    // 火焰警报
    document.getElementById('flame-alert').style.display =
      data.flame_detected ? 'block' : 'none';

    // 指标
    document.getElementById('fps').textContent = data.fps || 0;
    document.getElementById('photos').textContent = data.photo_count || 0;
    document.getElementById('timer').innerHTML = (data.timer||0).toFixed(1) + '<span class="unit">s</span>';
    document.getElementById('countdown').textContent = data.countdown > 0 ? data.countdown + 's' : '--';
    document.getElementById('uptime').innerHTML = Math.floor((data.uptime||0)/60) + '<span class="unit">min</span>';
    document.getElementById('update-time').textContent = data.timestamp || '--';
    document.getElementById('time').textContent = new Date().toLocaleTimeString();
  } catch(e) {
    console.error('refresh error', e);
  }
}

async function loadCaptures() {
  try {
    const res = await fetch('/api/captures');
    const data = await res.json();
    document.getElementById('cap-count').textContent = data.length;

    const html = data.slice(0, 20).map(f => `
      <div class="capture-item">
        <img src="/captures/${f.name}" loading="lazy">
        <div class="capture-info">
          <div class="capture-name">${f.name}</div>
          <div class="capture-meta">${f.time} · ${f.size_str}</div>
        </div>
        <a class="capture-dl" href="/captures/${f.name}" download>下载</a>
      </div>`).join('');
    document.getElementById('captures').innerHTML = html || '<div style="padding:20px;color:#666;text-align:center">暂无截图</div>';
  } catch(e) { console.error(e); }
}

async function loadLog() {
  try {
    const res = await fetch('/api/log');
    const text = await res.text();
    document.getElementById('log').textContent = text;
  } catch(e) {}
}

refresh(); loadCaptures(); loadLog();
setInterval(refresh, 3000);
setInterval(loadCaptures, 10000);
setInterval(loadLog, 5000);
</script>
</body>
</html>'''

# ====================== HTTP 处理器 ======================
class FireHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # 静默日志

    def send_data(self, data, content_type='text/html', code=200):
        if isinstance(data, str):
            data = data.encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/' or path == '/index.html':
            self.send_data(HTML_PAGE, 'text/html; charset=utf-8')

        elif path == '/api/status':
            status = read_status()
            self.send_data(json.dumps(status, ensure_ascii=False), 'application/json')

        elif path == '/api/captures':
            caps = get_captures()
            self.send_data(json.dumps(caps, ensure_ascii=False), 'application/json')

        elif path == '/api/log':
            log = get_log_tail()
            self.send_data(log, 'text/plain; charset=utf-8')

        elif path.startswith('/captures/'):
            filename = os.path.basename(path)
            filepath = os.path.join(CAPTURES_DIR, filename)
            if os.path.exists(filepath) and os.path.isfile(filepath):
                with open(filepath, 'rb') as f:
                    data = f.read()
                self.send_data(data, 'image/jpeg')
            else:
                self.send_data('Not Found', 'text/plain', 404)

        else:
            self.send_data('Not Found', 'text/plain', 404)


def main():
    import sys
    port = PORT
    if len(sys.argv) > 1:
        port = int(sys.argv[1])

    os.makedirs(CAPTURES_DIR, exist_ok=True)

    server = HTTPServer(('0.0.0.0', port), FireHandler)
    print(f'========================================')
    print(f' 消防通道检测系统 - Web 服务器')
    print(f'========================================')
    print(f'访问地址:')
    print(f'  本机:   http://localhost:{port}')
    print(f'  局域网: http://192.168.43.231:{port}')
    print(f'  手机:   连接同WiFi，浏览器打开上方地址')
    print(f'========================================')
    print(f'按 Ctrl+C 停止')
    print()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n服务器已停止')
        server.server_close()


if __name__ == '__main__':
    main()
