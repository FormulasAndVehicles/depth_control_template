#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""

import rclpy
from hippo_control_msgs.msg import ActuatorSetpoint
from hippo_msgs.msg import DepthStamped, Float64Stamped
from rclpy.node import Node


class DepthControlNode(Node):
    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0

        self.t_prev: rclpy.time.Time = self.get_clock().now()
        self.error_integral = 0.0
        self.error_prev = 0.0

        self.thrust_pub = self.create_publisher(
            msg_type=ActuatorSetpoint, topic='thrust_setpoint', qos_profile=1
        )

        self.setpoint_sub = self.create_subscription(
            msg_type=Float64Stamped,
            topic='depth_setpoint',
            callback=self.on_setpoint,
            qos_profile=1,
        )
        self.depth_sub = self.create_subscription(
            msg_type=DepthStamped,
            topic='depth',
            callback=self.on_depth,
            qos_profile=1,
        )

    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        current_depth = depth_msg.depth

        self.get_logger().info(
            f"Hi! I'm your controller running. "
            f'I received a depth of {current_depth} m.',
            throttle_duration_sec=1,
        )

        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # timestamp = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)
        thrust = self.compute_control_output(current_depth, timestamp)
        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)

    def publish_vertical_thrust(
        self, thrust: float, timestamp: rclpy.time.Time
    ) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    # t_now only needs to be added *after* the pure P control.
    # only I and D depend on time. For reliable results, the time should be
    # based on the input message stamp.
    def compute_control_output(
        self, current_depth: float, t_now: rclpy.time.Time
    ) -> float:
        # point out the problems with I and D:
        # D: very susceptible to noise, which puts high demands on the
        #   smoothness of input data.
        # I: often some integral windup leads to undesired behaviour.
        #   mitigation strategies: limit the integral term, only add to the
        #   integral when close to the setpoint, etc.
        #   a lot of heuristic adapations required perform well.
        error = self.current_setpoint - current_depth

        dt = (t_now - self.t_prev).nanoseconds * 1e-9

        # guard against time jumps. This will break the controller, since
        # it takes a very long time until the integral is back close to zero.
        if dt < 1.0:
            self.error_integral += error * dt

        # guard against to small timesteps
        if dt > 0.01:
            derror = (error - self.error_prev) / dt
        else:
            derror = 0.0

        self.t_prev = t_now
        self.error_prev = error

        # huge oscillations but follows setpoint somehow. large steady state
        # error
        p_gain = 3.0
        i_gain = 0.0
        d_gain = 0.0

        # huge d_gain supresses oscillations (probably not realistic for real
        # world setups)
        # damping causes the vehicle to move rather slowly
        p_gain = 3.0
        i_gain = 0.0
        d_gain = 5.0

        # huge p and d gain. looks great in the plot. Probably the real vehicle
        # is not able to perform in this way
        # also note that due to high p gain, the steady state error gets smaller
        # but will only vanish for infinite p gain.
        p_gain = 20.0
        i_gain = 0.0
        d_gain = 5.0

        # vehicle moves rather slowly again, but due to added integral term
        # the steady state error clearly vanishes.
        p_gain = 3.0
        i_gain = 1.0
        d_gain = 5.0

        u = p_gain * error + i_gain * self.error_integral + d_gain * derror
        thrust_z = u  # This doesn't seem right yet...
        return thrust_z


def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
