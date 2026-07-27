#!/usr/bin/env bash
# Bring up ft_sensor_node ROBUSTLY. The OptoForce DAQ box's UDP service (49152)
# often isn't ready immediately after a host reboot, and the node dies on the
# first refused connection instead of waiting. This first PROBES the box to tell
# apart "service down -> needs power-cycle" from "slow boot -> retry works",
# then retries the node with backoff until /ft_sensor/wrench publishes.
set -o pipefail
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ws_ctrl/install/setup.bash 2>/dev/null
BOX=192.168.0.5; PORT=49152

echo "probe: is the OptoForce UDP service listening on $BOX:$PORT?"
probe() {  # returns 0 if the box answers the real data-request command
  python3 - "$BOX" "$PORT" <<'PY'
import socket,sys
b,p=sys.argv[1],int(sys.argv[2])
try:
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); s.settimeout(2); s.connect((b,p))
    s.send(bytes.fromhex('1234000200000000')); s.recv(64); sys.exit(0)
except Exception: sys.exit(1)
PY
}
if ! probe; then
  echo "  box service DOWN (no UDP listener). This needs a PHYSICAL fix:"
  echo "    1) reseat the sensor cable at the DAQ box AND the wrist sensor"
  echo "    2) power-cycle the DAQ box, wait ~60s"
  echo "    3) re-run:  bash scripts/ft_connect.sh"
  echo "  (ping works because the network chip is up; the force firmware is not.)"
  exit 2
fi
echo "  box answers — starting ft_sensor_node with retry"
pkill -f ft_sensor_node 2>/dev/null; sleep 2
for try in 1 2 3 4 5; do
  setsid nohup bash -c 'source /opt/ros/humble/setup.bash && source ~/ws_ctrl/install/setup.bash && exec ros2 run magpie_control ft_sensor_node' >/tmp/log_ft.txt 2>&1 </dev/null &
  disown -a 2>/dev/null; sleep $((4*try))
  if timeout 8 ros2 topic echo /ft_sensor/wrench --once >/dev/null 2>&1; then
    echo "  ✓ FT publishing (attempt $try) — wrist_fz is live"; exit 0
  fi
  echo "  attempt $try: not publishing yet, retrying..."; pkill -f ft_sensor_node 2>/dev/null; sleep 2
done
echo "  ✗ box answers the probe but the node won't publish — check /tmp/log_ft.txt"
tail -3 /tmp/log_ft.txt; exit 1
