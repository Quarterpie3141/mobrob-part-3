#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from example_interfaces.srv import SetBool  # replace with custom srv ideally
from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    ERROR = auto()

# Define which transitions are legal
VALID_TRANSITIONS = {
    State.IDLE:    {State.RUNNING, State.ERROR},
    State.RUNNING: {State.PAUSED, State.IDLE, State.ERROR},
    State.PAUSED:  {State.RUNNING, State.IDLE, State.ERROR},
    State.ERROR:   {State.IDLE},
}


class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self._state = State.IDLE

        # Latched QoS so new subscribers get the current state immediately
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self._state_pub = self.create_publisher(String, '~/state', latched_qos)

        # Service for external nodes to command transitions.
        # Ideally define a custom srv: string requested_state -> bool success, string message
        self._transition_srv = self.create_service(
            SetBool, '~/request_transition', self._handle_transition_request
        )

        # Periodic state work (do_state_work)
        self._timer = self.create_timer(0.1, self._on_tick)

        self._publish_state()
        self.get_logger().info(f'State machine started in {self._state.name}')

    def _transition_to(self, new_state: State) -> tuple[bool, str]:
        if new_state == self._state:
            return True, f'Already in {new_state.name}'
        if new_state not in VALID_TRANSITIONS.get(self._state, set()):
            msg = f'Invalid transition: {self._state.name} -> {new_state.name}'
            self.get_logger().warn(msg)
            return False, msg

        old = self._state
        self._on_exit(old)
        self._state = new_state
        self._on_enter(new_state)
        self._publish_state()
        self.get_logger().info(f'Transitioned: {old.name} -> {new_state.name}')
        return True, 'OK'

    def _on_enter(self, state: State):
        # Hook: actions to run when entering a state
        pass

    def _on_exit(self, state: State):
        # Hook: cleanup when leaving a state
        pass

    def _on_tick(self):
        # Per-state periodic behavior
        if self._state == State.RUNNING:
            pass  # do work
        elif self._state == State.ERROR:
            pass  # maybe try recovery

    def _publish_state(self):
        msg = String()
        msg.data = self._state.name
        self._state_pub.publish(msg)

    def _handle_transition_request(self, request, response):
        # With SetBool this is just a toggle example; use a custom srv with a string field.
        target = State.RUNNING if request.data else State.IDLE
        ok, msg = self._transition_to(target)
        response.success = ok
        response.message = msg
        return response


def main():
    rclpy.init()
    node = StateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()