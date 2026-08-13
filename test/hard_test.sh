#!/usr/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

NODE_NAME="/franka_lock_unlock_node"
OUTER_LOOPS=2        # Full cycles (configure -> act/deact toggles -> cleanup)
INNER_LOOPS=2       # Number of Activate / Deactivate toggles per session
SLEEP_TIME=0.5       # Delay between state transitions in seconds

echo "=================================================="
echo "Starting Rigorous Lifecycle Test"
echo "Target Node:    $NODE_NAME"
echo "Outer Loops:    $OUTER_LOOPS (Configure/Cleanup cycles)"
echo "Inner Loops:    $INNER_LOOPS (Activate/Deactivate toggles)"
echo "=================================================="

# 1. Verify node availability
if ! ros2 lifecycle get "$NODE_NAME" > /dev/null 2>&1; then
    echo "[ERROR] Node '$NODE_NAME' is not running or unreachable!"
    exit 1
fi

echo "[INFO] Initial state: $(ros2 lifecycle get "$NODE_NAME")"

# 2. Outer Loop: Configure & Cleanup
for ((i=1; i<=OUTER_LOOPS; i++)); do
    echo ""
    echo "========================================="
    echo ">>> Outer Iteration $i / $OUTER_LOOPS"
    echo "========================================="

    # --- CONFIGURE ---
    echo "[+] Transitioning: CONFIGURE"
    ros2 lifecycle set "$NODE_NAME" configure
    sleep "$SLEEP_TIME"
    echo "    Current State: $(ros2 lifecycle get "$NODE_NAME")"

    # --- INNER LOOP: ACTIVATE / DEACTIVATE TOGGLES ---
    for ((j=1; j<=INNER_LOOPS; j++)); do
        echo "  --- Inner Loop $j / $INNER_LOOPS ---"

        # ACTIVATE
        echo "  [+] Transitioning: ACTIVATE"
        ros2 lifecycle set "$NODE_NAME" activate
        sleep "$SLEEP_TIME"
        echo "      Current State: $(ros2 lifecycle get "$NODE_NAME")"

        # DEACTIVATE
        echo "  [+] Transitioning: DEACTIVATE"
        ros2 lifecycle set "$NODE_NAME" deactivate
        sleep "$SLEEP_TIME"
        echo "      Current State: $(ros2 lifecycle get "$NODE_NAME")"
    done

    # --- CLEANUP ---
    echo "[+] Transitioning: CLEANUP"
    ros2 lifecycle set "$NODE_NAME" cleanup
    sleep "$SLEEP_TIME"
    echo "    Current State: $(ros2 lifecycle get "$NODE_NAME")"
done

# 3. Shutdown
echo ""
echo "========================================="
echo "[+] All outer and inner loops completed!"
echo "[+] Transitioning: SHUTDOWN"
ros2 lifecycle set "$NODE_NAME" shutdown
sleep "$SLEEP_TIME"
echo "    Final State: $(ros2 lifecycle get "$NODE_NAME")"
echo "========================================="
