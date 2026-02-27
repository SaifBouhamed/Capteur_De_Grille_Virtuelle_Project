#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class RobotSupervisor(Node):
    def __init__(self):
        super().__init__('robot_supervisor')

        self.gesture_sub = self.create_subscription(String, '/gesture_cmd', self.gesture_callback, 10)
        self.human_sub = self.create_subscription(Bool, '/human_detected', self.human_callback, 10)
        self.mode_pub = self.create_publisher(String, '/robot_operation_mode', 10)

        # States: EXPLORING | LISTENING
        self.state = 'EXPLORING'

        # --- Human presence tracking (anti-glitch) ---
        self.last_human_true_time = self.get_clock().now()
        self.human_present_window_s = 3.0

        # Exit conditions (seconds)
        self.human_lost_exit_s = 30.0
        self.no_gesture_exit_s = 30.0

        # Track last gesture time (for inactivity timeout)
        self.last_gesture_time = self.get_clock().now()

        # --- Victory confirmation (avoid 1-frame triggers) ---
        self.victory_confirm_s = 0.4
        self.pending_victory = False
        self.victory_first_time = None

        # --- Startup start burst (avoid race) ---
        self.start_burst_remaining = 6  # 6 * 0.5 = 3s
        self.start_burst_timer = self.create_timer(0.5, self.start_burst_tick)

        # Main tick
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info("Supervisor ready. State=EXPLORING (start burst enabled)")

    # -------------------- helpers --------------------
    def now(self):
        return self.get_clock().now()

    def secs_since(self, t):
        return (self.now() - t).nanoseconds / 1e9

    def publish_mode(self, value: str):
        m = String()
        m.data = value
        self.mode_pub.publish(m)

    def human_present(self) -> bool:
        return self.secs_since(self.last_human_true_time) <= self.human_present_window_s

    def human_lost_long(self) -> bool:
        return self.secs_since(self.last_human_true_time) >= self.human_lost_exit_s

    def no_gesture_long(self) -> bool:
        return self.secs_since(self.last_gesture_time) >= self.no_gesture_exit_s

    def switch_to_exploring(self, reason: str):
        if self.state != 'EXPLORING':
            self.state = 'EXPLORING'
        self.start_burst_remaining = 4
        if self.start_burst_timer.is_canceled():
            self.start_burst_timer = self.create_timer(0.5, self.start_burst_tick)
        self.publish_mode('start')
        self.get_logger().info(f"{reason} -> EXPLORING")

    # -------------------- startup --------------------
    def start_burst_tick(self):
        if self.state == 'EXPLORING' and self.start_burst_remaining > 0:
            self.publish_mode('start')
            self.start_burst_remaining -= 1
            if self.start_burst_remaining == 0:
                self.get_logger().info("Startup start-burst finished.")
        elif self.start_burst_remaining <= 0:
            self.start_burst_timer.cancel()

    # -------------------- callbacks --------------------
    def human_callback(self, msg: Bool):
        if msg.data:
            self.last_human_true_time = self.now()

    def gesture_callback(self, msg: String):
        cmd = msg.data.lower().strip()

        valid_cmds = {"stop", "victory", "peace", "left", "right", "forward", "backward"}
        if cmd in valid_cmds:
            self.last_gesture_time = self.now()

        # Explicit stop gesture: ALWAYS return to exploring
        if cmd == "stop":
            self.pending_victory = False
            self.victory_first_time = None
            self.switch_to_exploring("STOP gesture")
            return

        # Victory/peace: arm confirmation (only if human present)
        if cmd in ("victory", "peace"):
            if not self.human_present():
                self.pending_victory = False
                self.victory_first_time = None
                return

            if not self.pending_victory:
                self.pending_victory = True
                self.victory_first_time = self.now()
            return

        # Movement commands are forwarded only while LISTENING
        if self.state != 'LISTENING':
            return

        if cmd in ("left", "right", "forward", "backward"):
            self.publish_mode(cmd)

    # -------------------- supervisor loop --------------------
    def tick(self):
        # Confirm victory -> LISTENING
        if self.pending_victory and self.victory_first_time is not None:
            if self.secs_since(self.victory_first_time) >= self.victory_confirm_s:
                if self.human_present():
                    self.state = 'LISTENING'
                    self.publish_mode('listen')
                    self.get_logger().info("Victory confirmed -> LISTENING")
                    # Reset inactivity timer when entering listening
                    self.last_gesture_time = self.now()
                self.pending_victory = False
                self.victory_first_time = None

        # Exit LISTENING conditions:
        if self.state == 'LISTENING':
            if self.human_lost_long():
                self.switch_to_exploring("No person for 30s")
            elif self.no_gesture_long():
                self.switch_to_exploring("No gesture for 30s")


def main(args=None):
    rclpy.init(args=args)
    node = RobotSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
