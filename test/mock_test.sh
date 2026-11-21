#!/usr/bin/env bash
# Test the lifecycle node without a robot, with mock Desks (test/mock_desk.py).
#
# Usage (workspace with franka_lock_unlock built and sourced):
#   ./test/mock_test.sh [number of robots] [mock argument for the last robot]
#   ./test/mock_test.sh 2 --fail-unlock
#
# The node starts with autostart_state 'active' (self-test wait shortened to
# 1 s), then stops with SIGTERM: check that all robots are unlocked, then locked
# and logged out.
set -u
# Local DDS, no router needed. With rmw_zenoh_cpp, SIGTERM to the node also
# reaches this script.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ROBOTS=${1:-1}
LAST_ARG=${2:-}
DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
TMP=$(mktemp -d)
trap 'kill $(jobs -p) 2> /dev/null; rm -rf "$TMP"' EXIT
openssl req -x509 -newkey rsa:2048 -nodes -keyout "$TMP/key.pem" -out "$TMP/cert.pem" \
    -days 1 -subj /CN=mock 2> /dev/null

HOSTNAMES=""
for ((i = 1; i <= ROBOTS; i++)); do
    ARG=""
    if ((i == ROBOTS)); then ARG=$LAST_ARG; fi
    # shellcheck disable=SC2086
    python3 "$DIR/mock_desk.py" "127.0.0.$((i + 1))" 8443 "$TMP/cert.pem" "$TMP/key.pem" $ARG \
        > "$TMP/desk$i.log" 2>&1 &
    HOSTNAMES="$HOSTNAMES${HOSTNAMES:+, }127.0.0.$((i + 1)):8443"
done
sleep 1

cat > "$TMP/run_node.py" << 'PY'
import sys
import franka_lock_unlock.franka_client as fc
fc.SELF_TEST_WAIT = 1
from franka_lock_unlock.nodes.franka_lock_unlock_node import main
sys.exit(main())
PY
python3 "$TMP/run_node.py" --ros-args -p "hostnames:=[$HOSTNAMES]" -p password:=x \
    -p autostart_state:=active > "$TMP/node.log" 2>&1 &
NODE=$!
sleep $((ROBOTS * 2 + 25))
kill -TERM $NODE
wait $NODE

echo "=== node"
grep "franka_lock_unlock_node\]" "$TMP/node.log" | sed 's/^.*\]: //'
for ((i = 1; i <= ROBOTS; i++)); do
    echo "=== desk $i"
    grep "STATE\|logout" "$TMP/desk$i.log"
done
