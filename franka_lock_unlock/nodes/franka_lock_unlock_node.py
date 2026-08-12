import rclpy
from rclpy.timer import Timer
from rclpy.lifecycle import Node, State, TransitionCallbackReturn, LifecycleState
from rcl_interfaces.msg import ParameterType, ParameterValue

from franka_lock_unlock.franka_lock_unlock import FrankaLockUnlock

RESET_TIME_IN_SECS = 23*60*60 # NOTE: the token expires after 24 hours

class FrankLockUnlockNode(Node):
    """Franka Lock Unlock ROS2 Lifecycle Node."""
    def __init__(self):
        super().__init__("franka_lock_unlock_node")

        self.declare_parameter('hostname', ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='localhost'))
        self.declare_parameter('username', ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='admin'))
        self.declare_parameter('password', ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='strongpwd'))
        self.declare_parameter('enable_relock', ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
        self.declare_parameter('wait_web_ui', ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
        self.declare_parameter('request_physical_access', ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
        self.declare_parameter('enable_fci', ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))
        self.declare_parameter('robot_type', ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='fr3'))

        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        self.username= self.get_parameter('username').get_parameter_value().string_value
        self.password= self.get_parameter('password').value
        self.enable_relock = self.get_parameter('enable_relock').get_parameter_value().bool_value
        self.wait_web_ui= self.get_parameter('wait_web_ui').get_parameter_value().bool_value
        self.request_physical_access= self.get_parameter('request_physical_access').get_parameter_value().bool_value
        self.enable_fci= self.get_parameter('enable_fci').get_parameter_value().bool_value
        self.robot_type= self.get_parameter('robot_type').value
        self.get_logger().debug(f"hostname: {self.hostname}, username: {self.username}, relock {self.enable_relock}, wait web ui: {self.wait_web_ui}, request physical access: {self.request_physical_access}, enable fci: {self.enable_fci}, robot type: {self.robot_type}")

        # NOTE: resetting is handled by Node itself when the shutdown trigger is called.
        self.franka_lock_unlock = FrankaLockUnlock(hostname=self.hostname, username=self.username, password=self.password)

        self.timer: Timer | None = None
        self._current_state = 'unconfigured'
        self._resetting_token = False

        self.get_logger().info(f"{self.get_name()} node started.")

    def _start_timer(self):
        """Safely start the timer"""
        if self.timer is None:
            self.timer = self.create_timer(RESET_TIME_IN_SECS, self.trigger_reset)
            self.get_logger().info(
                f"Reset timer created (triggers every {RESET_TIME_IN_SECS} seconds)."
            )
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
        if not self.hostname or not self.hostname.strip():
            self.get_logger().error("Parameter 'hostname' cannot be empty.")
            return False

        if not self.username or not self.username.strip():
            self.get_logger().error("Parameter 'username' cannot be empty.")
            return False

        if not self.password or not self.password.strip():
            self.get_logger().error("Parameter 'password' cannot be empty.")
            return False

        if self.request_physical_access and not self.wait_web_ui:
            self.get_logger().error(
                "Invalid configuration: 'request_physical_access' is True, "
                "but 'wait_web_ui' is False. (request requires wait)"
            )
            return False

        # TODO(Sachin): This is not required remove this later
        # if self.enable_fci and not self.enable_unlock:
        #     self.get_logger().error(
        #         "Invalid configuration: 'enable_fci' is True, "
        #         "but 'enable_unlock' is False. (fci requires unlock)"
        #     )
        #     return False
        return True

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Handles the configure state."""
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configure'"
        )
        # validate all the params
        valid_res = self._validate_params()
        if not valid_res:
            self.get_logger().error("Params are not valid. Please recheck them.")
            return TransitionCallbackReturn.ERROR

        # login
        res, msg = self.franka_lock_unlock.try_login(self.request_physical_access)
        if not res:
            self.get_logger().error(msg)
            return TransitionCallbackReturn.FAILURE
        self._start_timer()
        self._current_state = 'inactive'
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Handles the activate state."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'"
        )
        res, msg = self.franka_lock_unlock.try_lock_unlock(True, self.request_physical_access)
        if not res:
            self.get_logger().error(msg)
            return TransitionCallbackReturn.FAILURE
        self._current_state = 'active'
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Handles the deactivate state."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'"
        )

        res, msg = self.franka_lock_unlock.try_lock_unlock(False)
        if not res:
            self.get_logger().error(msg)
            return TransitionCallbackReturn.FAILURE
        self._current_state = 'inactive'
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Handles the shutdown state."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'"
        )
        self._stop_timer()
        if state.label == "active" or self.enable_relock:
            self.get_logger().info("Relocking robot before shutdown...")
            res, msg = self.franka_lock_unlock.try_lock_unlock(False)
            if not res:
                self.get_logger().error(msg)
                return TransitionCallbackReturn.FAILURE

        res, msg = self.franka_lock_unlock.try_logout()
        if not res:
            self.get_logger().warn(msg)
            return TransitionCallbackReturn.FAILURE
        self._current_state = 'shutdown'
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Handles the cleanup state."""
        if self._resetting_token:
            self.get_logger().error("Please wait token is resetting currently")
            return TransitionCallbackReturn.ERROR
        self.get_logger().info(
            f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'cleanup'"
        )

        self._stop_timer()

        res, msg = self.franka_lock_unlock.try_logout()
        if not res:
            self.get_logger().error(msg)
            return TransitionCallbackReturn.FAILURE
        self._current_state = 'unconfigured'
        return TransitionCallbackReturn.SUCCESS

    def trigger_shutdown(self):
        """Triggers the shutdown when keyboard interrupt stops the launch file."""
        # TODO(Sachin): Check if there is other approach to deal with this
        if not self.franka_lock_unlock.is_logged_in():
            self.get_logger().info("Not logged in, nothing to clean up.")
            return
        if self.enable_relock:
            res, msg = self.franka_lock_unlock.try_lock_unlock(False)
            if not res:
                self.get_logger().error(msg)
        res, msg = self.franka_lock_unlock.try_logout()
        if not res:
            self.get_logger().error(msg)

    def trigger_reset(self):
        """Triggers the reset before 24 hours."""
        self.get_logger().info("Refreshing Franka session before token expiry...")
        self._resetting_token = True

        was_active = (self._current_state == 'active')

        if was_active:
            res, msg = self.franka_lock_unlock.try_lock_unlock(False)
            if not res:
                self.get_logger().error(f"Reset: lock failed: {msg}")
                return

        res, msg = self.franka_lock_unlock.try_logout()
        if not res:
            self.get_logger().error(f"Reset: logout failed: {msg}")
            return

        res, msg = self.franka_lock_unlock.try_login(self.request_physical_access)
        if not res:
            self.get_logger().error(f"Reset: re-login failed: {msg}")
            return

        if was_active:
            res, msg = self.franka_lock_unlock.try_lock_unlock(True, self.request_physical_access)
            if not res:
                self.get_logger().error(f"Reset: re-unlock failed: {msg}")
                return

        self._resetting_token = False
        self.get_logger().info("Franka session refreshed successfully.")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = FrankLockUnlockNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SIGINT received. Triggering lifecycle shutdown...")
        node.trigger_shutdown()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
