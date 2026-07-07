#!/bin/bash
# install_services.sh - 一键部署 systemd 服务
# 用法: sudo bash install_services.sh

set -e

PROJECT_DIR="/home/user/fire_detect_project"

echo "===== 安装 systemd 服务 ====="

# 1. 复制服务文件
cp $PROJECT_DIR/scripts/openamp-init.service /etc/systemd/system/
cp $PROJECT_DIR/scripts/fire-detect.service /etc/systemd/system/
cp $PROJECT_DIR/scripts/fire-web.service /etc/systemd/system/

# 2. 设置脚本权限
chmod +x $PROJECT_DIR/scripts/openamp-start.sh
chmod +x $PROJECT_DIR/scripts/install_services.sh

# 3. 重载 systemd
systemctl daemon-reload

# 4. 启用开机自启
systemctl enable openamp-init.service
systemctl enable fire-detect.service
systemctl enable fire-web.service

echo "===== 安装完成 ====="
echo "服务启动顺序: openamp-init → fire-detect → fire-web"
echo ""
echo "常用命令:"
echo "  sudo systemctl start fire-detect   # 立即启动检测"
echo "  sudo systemctl stop fire-detect    # 停止检测"
echo "  sudo systemctl status fire-detect  # 查看状态"
echo "  sudo journalctl -u fire-detect -f  # 实时日志"
echo "  sudo systemctl restart fire-web    # 重启Web"
echo ""
echo "重启板子后三个服务会自动启动"
