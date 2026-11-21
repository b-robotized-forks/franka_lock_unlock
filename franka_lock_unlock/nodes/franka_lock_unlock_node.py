from concurrent.futures import ThreadPoolExecutor
from typing import Callable

import requests
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.timer import Timer
from rclpy.lifecycle import Node, State, TransitionCallbackReturn


from franka_lock_unlock.franka_lock_unlock_params import franka_lock_unlock
from franka_lock_unlock.franka_lock_unlock import FrankaLockUnlock

HOURS_TO_RESET = 23
RESET_TIME_IN_SECS = HOURS_TO_RESET * 60 * 60  # NOTE: the token expires after 24 hours
TIME_IN_SECS_TO_CHECK_DESK = 5 * 60
DESK_CHECK_TIMEOUT = 3.0  # seconds

Result = tuple[bool, str]


class FrankLockUnlockNode(Node):
    """
    Franka Lock Unlock ROS2 Lifecycle Node for one or more robots.

    All robots share the parameters and the lifecycle state: each transition runs on all robots
    in parallel. When it fails on one robot, the node returns the other robots to the previous
    state.
    """

    def __init__(self):
        super().__init__("franka_lock_unlock_node")

        self.param_listener = franka_lock_unlock.ParamListener(self)
        self.params = self.param_listener.get_params()

        self.get_logger().debug(
            f"hostnames: {self.params.hostnames}, username: {self.params.username}, relock {self.params.enable_relock}, wait web ui: {self.params.wait_web_ui}, request physical access: {self.params.request_physical_access}, enable fci: {self.params.enable_fci}, robot type: {self.params.robot_type}"
        )

        # NOTE: resetting is handled by Node itself when the shutdown trigger is called.
        self.robots = {
            hostname: FrankaLockUnlock(
                hostname=hostname,
                username=self.params.username,
                password=self.params.password,
                robot_type=self.params.robot_type,
            )
            for hostname in self.params.hostnames
        }
        self._executor = ThreadPoolExecutor(max_workers=len(self.robots))

        self.timer: Timer | None = None
        self._current_state = "unconfigured"
        self._resetting_token = False

        # Check that Franka Desk answers. No ICMP: 'ping' is not available in a strict snap.
        for hostname in self.robots:
            self._check_desk(hostname, log_success=True)
        self.desk_check_timer = self.create_timer(
            TIME_IN_SECS_TO_CHECK_DESK, self._check_desk_callback
        )

        self.get_logger().info(
            f"{self.get_name()} node started for {len(self.robots)} robot(s): "
            f"{', '.join(self.robots)}."
        )

    def _run(
        self,
        action: str,
        func: Callable[[FrankaLockUnlock], Result],
        hostnames: list[str] | None = None,
    ) -> list[str]:
        """
        Run 'func' on the robots in parallel and log the results.

        Returns
        -------
        list[str]:
            Hostnames of the robots on which 'func' succeeded.

        """
        hostnames = list(self.robots) if hostnames is None else hostnames
        futures = {h: self._executor.submit(func, self.robots[h]) for h in hostnames}
        succeeded = []
        for hostname, future in futures.items():
            try:
                res, msg = future.result()
            except Exception as e:
                res, msg = False, f"Exception: {e}"
            if res:
                succeeded.append(hostname)
                self.get_logger().info(f"[{hostname}] {action}: {msg}")
            else:
                self.get_logger().error(f"[{hostname}] {action} failed: {msg}")
        return succeeded

    def _run_all(self, action: str, func: Callable[[FrankaLockUnlock], Result]) -> bool:
        """Run 'func' on all robots in parallel; True if it succeeded on all robots."""
        return len(self._run(action, func)) == len(self.robots)

    def _lock(self, robot: FrankaLockUnlock) -> Result:
        return robot.try_lock_unlock(False)

    def _unlock(self, robot: FrankaLockUnlock) -> Result:
        return robot.try_lock_unlock(
            True, self.params.request_physical_access, enable_fci=self.params.enable_fci
        )

    def _login(self, robot: FrankaLockUnlock) -> Result:
        return robot.try_login(
            self.params.request_physical_access,
            self.params.wait_web_ui,
            self.params.physical_access_timeout,
        )

    def _logout(self, robot: FrankaLockUnlock) -> Result:
        return robot.try_logout()

    def _self_test(self, robot: FrankaLockUnlock) -> Result:
        return robot.try_acknowledge_and_execute_self_test()

    def _check_desk(self, hostname: str, log_success: bool = False) -> None:
        """Send an HTTPS request to Franka Desk (no login) and warn if it does not answer."""
        url = f"https://{hostname}/"
        try:
            response = requests.get(url, timeout=DESK_CHECK_TIMEOUT, verify=False)
            if log_success:
                self.get_logger().info(
                    f"Franka Desk at '{url}' answers (HTTP {response.status_code})."
                )
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f"Franka Desk at '{url}' does not answer: {e}")

    def _check_desk_callback(self):
        for hostname in self.robots:
            self._check_desk(hostname)

    def _start_timer(self):
        """Safely start the timer."""
        if self.timer is None:
            self.timer = self.create_timer(RESET_TIME_IN_SECS, self.trigger_reset)
            self.get_logger().info(f"Reset timer created (triggers every {HOURS_TO_RESET} hours).")
        else:
            self.timer.reset()

    def _stop_timer(self):
        """Safely stops and destroys the reset timer."""
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
            self.get_logger().info("Reset timer stopped.")

    def _validate_params(self) -> bool:
        """Validate the params."""
        if not all(h.strip() for h in self.params.hostnames):
            self.get_logger().error("Parameter 'hostnames' cannot contain empty entries.")
            return False

        if not self.params.username.strip():
            self.get_logger().error("Parameter 'username' cannot be empty or whitespace-only.")
            return False

        if not self.params.password.strip():
            self.get_logger().error("Parameter 'password' cannot be empty or whitespace-only.")
            return False

        if self.params.request_physical_access and not self.params.wait_web_ui:
            self.get_logger().error(
                "Invalid configuration: 'request_physical_access' is True, "
                "but 'wait_web_ui' is False. (request requires wait)"
            )
            return False
        return True

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Log in to all robots, take the control tokens and run the self-tests."""
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'"
        )
        if not self._validate_params():
            self.get_logger().error("Params are not valid. Please recheck them.")
            return TransitionCallbackReturn.ERROR

        if not self._run_all("login", self._login):
            # Also the robots with a failed login: release the token request and log out.
            self._run_all("logout", self._logout)
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Running self-test")
        if not self._run_all("self-test", self._self_test):
            self._run_all("logout", self._logout)
            return TransitionCallbackReturn.FAILURE

        self._start_timer()
        self._current_state = "inactive"
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Unlock the brakes of all robots and activate the FCI."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'"
        )
        unlocked = self._run("unlock", self._unlock)
        if len(unlocked) != len(self.robots):
            self.get_logger().error("Unlock failed on a robot. Locking the other robots again.")
            self._run("lock", self._lock, unlocked)
            return TransitionCallbackReturn.FAILURE
        self._current_state = "active"
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Lock the brakes of all robots."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'"
        )
        if not self._run_all("lock", self._lock):
            return TransitionCallbackReturn.FAILURE
        self._current_state = "inactive"
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handle the shutdown state."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'"
        )
        self._stop_timer()
        if state.label == "active" or self.params.enable_relock:
            self.get_logger().info("Relocking robots before shutdown...")
            if not self._run_all("lock", self._lock):
                return TransitionCallbackReturn.FAILURE

        if not self._run_all("logout", self._logout):
            return TransitionCallbackReturn.FAILURE
        self._current_state = "shutdown"
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Handle the cleanup state."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'cleanup'"
        )

        self._stop_timer()

        if not self._run_all("logout", self._logout):
            return TransitionCallbackReturn.FAILURE
        self._current_state = "unconfigured"
        return TransitionCallbackReturn.SUCCESS

    def release_on_exit(self):
        """Relock (if enabled) and log out of all robots when the process stops."""
        logged_in = [h for h, robot in self.robots.items() if robot.is_logged_in()]
        if not logged_in:
            self.get_logger().info("Not logged in, nothing to clean up.")
            return
        if self.params.enable_relock:
            # Lock only the robots that this node controls.
            with_control = [h for h in logged_in if self.robots[h].has_control()]
            self._run("lock", self._lock, with_control)
        self._run("logout", self._logout, logged_in)

    def trigger_reset(self):
        """Triggers the reset before 24 hours."""
        self.get_logger().info("Refreshing Franka sessions before token expiry...")
        self._resetting_token = True
        try:
            self._refresh_sessions()
        finally:
            self._resetting_token = False

    def _refresh_sessions(self):
        """Log out and in again, run the self-test and restore the unlocked state."""
        was_active = self._current_state == "active"

        if was_active and not self._run_all("lock", self._lock):
            self.get_logger().error("Reset: lock failed.")
            return
        if not self._run_all("logout", self._logout):
            self.get_logger().error("Reset: logout failed.")
            return
        if not self._run_all("login", self._login):
            self.get_logger().error("Reset: re-login failed.")
            return
        if not self._run_all("self-test", self._self_test):
            self.get_logger().error("Reset: self-test failed.")
            return
        if was_active and not self._run_all("unlock", self._unlock):
            self.get_logger().error("Reset: re-unlock failed.")
            return

        self.get_logger().info("Franka sessions refreshed successfully.")


def main(args=None):
    # rclpy handles SIGINT and SIGTERM (e.g. 'snap stop'): both relock and log out.
    rclpy.init(args=args)
    node = None
    try:
        node = FrankLockUnlockNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.get_logger().info("Stop signal received. Relocking and logging out...")
            node.release_on_exit()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
