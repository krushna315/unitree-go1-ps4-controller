#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_IP="192.168.12.1"
IFACE="wlxdc6279663481"
SSID="Unitree_Go421436A"

echo "[1/4] Connecting to Wi-Fi $SSID..."
nmcli connection up "$SSID" ifname "$IFACE" 2>/dev/null || true

echo "[2/4] Waiting for robot at $ROBOT_IP to respond..."
while ! ping -c 1 -W 1 "$ROBOT_IP" >/dev/null 2>&1; do
    nmcli connection up "$SSID" ifname "$IFACE" 2>/dev/null || true
    sleep 2
done
echo "Robot is ONLINE!"

echo "[3/4] Ensuring target directory exists and copying pure UDP controller script..."
ssh pi@$ROBOT_IP "mkdir -p /home/pi/Unitree/ps4_controller"
scp -O "$SCRIPT_DIR/unitree_bms.cpython-37m-aarch64-linux-gnu.so" pi@$ROBOT_IP:/home/pi/Unitree/ps4_controller/unitree_bms.cpython-37m-aarch64-linux-gnu.so
scp -O "$SCRIPT_DIR/unitree_go1_ps4_controller.py" pi@$ROBOT_IP:/home/pi/Unitree/ps4_controller/unitree_go1_ps4_controller.py
scp -O "$SCRIPT_DIR/unitree_go1_ps4.service" pi@$ROBOT_IP:/home/pi/Unitree/ps4_controller/unitree_go1_ps4.service
scp -O "$SCRIPT_DIR/README.md" pi@$ROBOT_IP:/home/pi/Unitree/ps4_controller/README.md

echo "[4/4] Updating systemd service and restarting..."
ssh pi@$ROBOT_IP "sudo cp /home/pi/Unitree/ps4_controller/unitree_go1_ps4.service /etc/systemd/system/unitree_go1_ps4.service && sudo systemctl daemon-reload && sudo systemctl restart unitree_go1_ps4.service"

echo "DONE! Pure UDP controller is live and running."
