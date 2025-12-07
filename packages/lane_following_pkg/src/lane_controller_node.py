#!/usr/bin/env python3

# OMAR

import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType
from geometry_msgs.msg import Point
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty, EmptyResponse

class LaneControllerNode(DTROS):
    def __init__(self, node_name):
        super(LaneControllerNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Declare and get parameters
        # Set default parameters on parameter server
        if not rospy.has_param('~k1'):
            rospy.set_param('~k1', 0.3)  # Reduced from 1.0
        if not rospy.has_param('~k2'):
            rospy.set_param('~k2', 0.3)  # Reduced from 1.0
        if not rospy.has_param('~k3'):
            rospy.set_param('~k3', 0.3)  # Reduced from 1.0
        if not rospy.has_param('~kp'):
            rospy.set_param('~kp', 0.001)  # Reduced from 2.0 - this is the proportional gain for midpoint error
        if not rospy.has_param('~ki'):
            rospy.set_param('~ki', 0.0005)  # Integral gain
        if not rospy.has_param('~kd'):
            rospy.set_param('~kd', 0.00001)  # Derivative gain
        if not rospy.has_param('~kp_angle'):
            rospy.set_param('~kp_angle', 0.01)  # Proportional gain for angle control
        if not rospy.has_param('~ki_angle'):
            rospy.set_param('~ki_angle', 0.001)  # Integral gain for angle control
        if not rospy.has_param('~kd_angle'):
            rospy.set_param('~kd_angle', 0.0001)  # Derivative gain for angle control
        if not rospy.has_param('~target_white_line_angle'):
            rospy.set_param('~target_white_line_angle', 51.0)  # Target angle in degrees
        if not rospy.has_param('~v'):
            rospy.set_param('~v', 0.005)  # forward speed (m/s)
        if not rospy.has_param('~L'):
            rospy.set_param('~L', 0.094)  # wheelbase (m)
        if not rospy.has_param('~R'):
            rospy.set_param('~R', 0.031)  # wheel radius (m)
        if not rospy.has_param('~omega_max'):
            rospy.set_param('~omega_max', 2.0)  # Reduced from 5.0 - limit maximum turning rate
        if not rospy.has_param('~wheel_speed_max'):
            rospy.set_param('~wheel_speed_max', 5.0)  # Reduced from 10.0
        if rospy.has_param('~vanishing_point_filter_alpha'):
            rospy.set_param('~vanishing_point_filter_alpha', 0.7)  # EMA filter: higher = more reactive (changed from 0.3)

        self.update_parameters()

        # Initialize feature values
        self.x_v = None  # Raw vanishing point x
        self.x_v_filtered = None  # Filtered vanishing point x (for control)
        self.x_m = None
        
        # PID controller state
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        
        # PID controller state for angle control
        self.integral_error_angle = 0.0
        self.last_error_angle = 0.0
        self.last_time_angle = None
        
        # Output enable/disable (controlled by services only)
        self.enable_output = False  # False at startup, only services can change this
        
        # Controller enable/disable (controlled by corner detection)
        self.controller_enabled = True  # Start enabled, corner detection will toggle this
        
        self.corner_detected = False  # True if corner was detected
        self.yellow_line_visible = True
        self.white_line_visible = True
        self.white_line_angle = 0.0

        # Cache parameters (read once at init, update periodically)
        self.update_parameters()
        self.param_update_counter = 0

        self._vehicle_name = os.environ.get('VEHICLE_NAME', 'deutschbot')

        # Publishers (create BEFORE subscribers to avoid AttributeError in callbacks)
        self.wheels_pub = rospy.Publisher(f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=10)
        self.omega_pub = rospy.Publisher(f"/{self._vehicle_name}/lane_controller/omega", Float64, queue_size=10)
        self.enabled_pub = rospy.Publisher(f"/{self._vehicle_name}/lane_controller/enabled", Bool, queue_size=10)
        
        # Publishers for tracking vanishing point (for rqt_plot)
        self.x_v_current_pub = rospy.Publisher(f"/{self._vehicle_name}/lane_controller/x_v_current", Float64, queue_size=10)
        self.x_v_target_pub = rospy.Publisher(f"/{self._vehicle_name}/lane_controller/x_v_target", Float64, queue_size=10)
        
        # Subscribers (create AFTER publishers so callbacks can use them)
        self.vanish_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/vanishing_point", Point, self.vanishing_callback, queue_size=10)
        self.mid_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/mid_point", Point, self.middle_callback, queue_size=10)

        self.corner_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/corner_detected", Bool, self.corner_callback, queue_size=10)
        self.yellow_line_visible_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/yellow_line_visible", Bool, self.yellow_line_visible_callback, queue_size=10)
        self.white_line_visible_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/white_line_visible", Bool, self.white_line_visible_callback, queue_size=10)
        self.white_line_angle_sub = rospy.Subscriber(f"/{self._vehicle_name}/lane_following/white_line_angle", Float64, self.white_line_angle_callback, queue_size=10)

        # Services for runtime control
        self.reset_srv = rospy.Service(f"/{self._vehicle_name}/lane_controller/reset", Empty, self.reset_callback)
        self.enable_srv = rospy.Service(f"/{self._vehicle_name}/lane_controller/enable", Empty, self.enable_callback)
        self.disable_srv = rospy.Service(f"/{self._vehicle_name}/lane_controller/disable", Empty, self.disable_callback)
        
        # Register shutdown hook to stop wheels when node dies
        rospy.on_shutdown(self.shutdown_hook)
        
        self.log("LaneControllerNode initialized.")


    def vanishing_callback(self, msg):
        # Apply exponential moving average (EMA) filter to reduce noise
        # x_filtered(n) = alpha * x_raw(n) + (1 - alpha) * x_filtered(n-1)
        # alpha close to 1 = less filtering, alpha close to 0 = more smoothing
        
        self.x_v = msg.x  # Store raw value
        
        if self.x_v_filtered is None:
            # First measurement - initialize filter
            self.x_v_filtered = self.x_v
        else:
            # Apply EMA filter
            self.x_v_filtered = self.vanishing_point_filter_alpha * self.x_v + (1.0 - self.vanishing_point_filter_alpha) * self.x_v_filtered
        
        self.try_compute_control()

    def middle_callback(self, msg):
        self.x_m = msg.x
        # Don't call try_compute_control here - only from vanishing_callback to avoid double updates
    
    def corner_callback(self, msg):
        prev_corner_detected = self.corner_detected
        self.corner_detected = msg.data

        if self.corner_detected and not prev_corner_detected:  # Check for transition from False to True
            self.log("Corner detected - disabling controller")
            self.controller_enabled = False
            self.try_compute_control()
        elif not self.corner_detected and prev_corner_detected:     # Check for transition from True to False
            self.log("Corner cleared - enabling controller")
            self.controller_enabled = True

    def yellow_line_visible_callback(self, msg):
        prev_yellow_line_visible = self.yellow_line_visible
        self.yellow_line_visible = msg.data

        if prev_yellow_line_visible and not self.yellow_line_visible:
            self.log("Yellow line not visible")
    
    def white_line_visible_callback(self, msg):
        self.white_line_visible = msg.data
        if not self.white_line_visible:
            self.log("WARNING: White line not visible - stopping!")

    def white_line_angle_callback(self, msg):
        self.white_line_angle = msg.data
    

    def update_parameters(self):
        """Update controller parameters from parameter server"""
        self.k1 = rospy.get_param('~k1', 0.5)
        self.k2 = rospy.get_param('~k2', 0.5)
        self.k3 = rospy.get_param('~k3', 0.5)
        self.L = rospy.get_param('~L', 0.094)
        self.R = rospy.get_param('~R', 0.031)
        self.omega_max = rospy.get_param('~omega_max', 2.0)
        self.wheel_speed_max = rospy.get_param('~wheel_speed_max', 5.0)
        self.vanishing_point_filter_alpha = rospy.get_param('~vanishing_point_filter_alpha', 0.3)
        self.update_ctrl_parameters()

    def update_ctrl_parameters(self):
        self.kp = rospy.get_param('~kp', 0.5)
        self.ki = rospy.get_param('~ki', 0.01)
        self.kd = rospy.get_param('~kd', 0.05)
        self.kp_angle = rospy.get_param('~kp_angle', 0.01)
        self.ki_angle = rospy.get_param('~ki_angle', 0.001)
        self.kd_angle = rospy.get_param('~kd_angle', 0.0001)
        self.target_white_line_angle = rospy.get_param('~target_white_line_angle', 51.0)
        self.v = rospy.get_param('~v', 0.0)


    def try_compute_control(self):
        # Update parameters every 100 calls (~3 seconds at 30Hz) instead of every call
        self.param_update_counter += 1
        if self.param_update_counter >= 100:
            self.update_ctrl_parameters()
            self.param_update_counter = 0
        
        # Check if output is disabled via service -> stop immediately
        if not self.enable_output:
            # Send zero command when output disabled
            cmd = WheelsCmdStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.vel_left = 0.0
            cmd.vel_right = 0.0
            return
        
        # Check if white line is missing -> stop immediately for safety
        if not self.white_line_visible:
            self.disable_callback(None)
            cmd = WheelsCmdStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.vel_left = 0.0
            cmd.vel_right = 0.0
            self.wheels_pub.publish(cmd)
            return
    

        # Set omega based on controller state
        if self.corner_detected:
            # Stop the robot when detecting a corner
            omega = 0.0
            self.v = 0.0

            self.controller_enabled = False
            self.reset_callback(None)
            self.disable_callback(None)

            self.log(f"Corner maneuver: setting constant omega={omega}")
        elif self.controller_enabled:
            if not self.yellow_line_visible:
                """ Test: PID control on white line angle only when yellow line is missing
                # Control angle of white line using PID controller
                error_angle = self.target_white_line_angle - self.white_line_angle

                # Calculate dt for integral and derivative terms
                current_time_angle = rospy.Time.now()
                if self.last_time_angle is None:
                    dt_angle = 0.0
                else:
                    dt_angle = (current_time_angle - self.last_time_angle).to_sec()
                self.last_time_angle = current_time_angle
                
                # Integral term with anti-windup
                if dt_angle > 0:
                    self.integral_error_angle += error_angle * dt_angle
                    # Anti-windup: limit integral term
                    max_integral_angle = self.omega_max / max(self.ki_angle, 1e-6)
                    self.integral_error_angle = max(-max_integral_angle, min(max_integral_angle, self.integral_error_angle))
                
                # Derivative term
                if dt_angle > 0:
                    derivative_angle = (error_angle - self.last_error_angle) / dt_angle
                else:
                    derivative_angle = 0.0
                self.last_error_angle = error_angle
                
                # PID control law for angle
                #omega = self.kp_angle * error_angle + self.ki_angle * self.integral_error_angle + self.kd_angle * derivative_angle
                self.log(f"Yellow line not visible - Angle PID: e={error_angle:.2f}°, I={self.integral_error_angle:.2f}, D={derivative_angle:.2f}, ω={omega:.3f}")
                """
                
                omega = 0.0         # Hold omega at 0 when yellow line is missing (simplification)
                
            else:
                # PID Controller on vanishing point error
                if self.x_v_filtered is None or self.x_m is None:
                    return

                # Error: we want vanishing point at center (x_v = 0)
                trgt_x_v = -30          # Target vanishing point offset to the left
                error = trgt_x_v - self.x_v_filtered
                
                # Publish current and target x_v for rqt_plot
                x_v_current_msg = Float64()
                x_v_current_msg.data = self.x_v_filtered
                self.x_v_current_pub.publish(x_v_current_msg)
                
                x_v_target_msg = Float64()
                x_v_target_msg.data = trgt_x_v
                self.x_v_target_pub.publish(x_v_target_msg)
                
                # Calculate dt for integral and derivative terms
                current_time = rospy.Time.now()
                if self.last_time is None:
                    dt = 0.0
                else:
                    dt = (current_time - self.last_time).to_sec()
                self.last_time = current_time
                
                # Integral term with anti-windup
                if dt > 0:
                    self.integral_error += error * dt
                    # Anti-windup: limit integral term
                    max_integral = self.omega_max / max(self.ki, 1e-6)
                    self.integral_error = max(-max_integral, min(max_integral, self.integral_error))
                
                # Derivative term
                if dt > 0:
                    derivative = (error - self.last_error) / dt
                else:
                    derivative = 0.0
                self.last_error = error

                omega = self.kp * error + self.ki * self.integral_error + self.kd * derivative

                self.log(f"PID: x_v={self.x_v_filtered:.1f}→{trgt_x_v:.1f}, e={error:.2f}, P={self.kp*error:.2f}, I={self.ki*self.integral_error:.2f}, D={self.kd*derivative:.2f}, ω={omega:.3f}")
        
        else:
            # This should never happen due to early return, but add safety
            return

        omega = max(-self.omega_max, min(self.omega_max, omega))

        # Publish omega control output
        omega_msg = Float64()
        omega_msg.data = omega
        self.omega_pub.publish(omega_msg)

        # Convert to wheel angular velocities (rad/s)
        # Differential drive equations: v = R*(w_r + w_l)/2, omega = R*(w_r - w_l)/L
        # Solving for w_l and w_r:
        v_l = (self.v - omega * self.L / 2) / self.R
        v_r = (self.v + omega * self.L / 2) / self.R

        v_l = max(-self.wheel_speed_max, min(self.wheel_speed_max, v_l))
        v_r = max(-self.wheel_speed_max, min(self.wheel_speed_max, v_r))

        # Publish
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = float(v_l)
        cmd.vel_right = float(v_r)
        self.wheels_pub.publish(cmd)
        
        # Publish controller enabled status
        enabled_msg = Bool()
        enabled_msg.data = self.controller_enabled
        self.enabled_pub.publish(enabled_msg)
    
    def reset_callback(self, req):
        """Service callback to reset PID controller state"""
        self.log("Resetting PID controller state")
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.integral_error_angle = 0.0
        self.last_error_angle = 0.0
        self.last_time_angle = None
        return EmptyResponse()
    
    def enable_callback(self, req):
        """Service callback to enable output"""
        self.log("Enabling output")
        self.enable_output = True
        self.reset_callback(None)  # Reset PID state when enabling
        return EmptyResponse()
    
    def disable_callback(self, req):
        """Service callback to disable output"""
        self.log("Disabling output")
        self.enable_output = False
        # Reset PID state when disabling
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None
        self.integral_error_angle = 0.0
        self.last_error_angle = 0.0
        self.last_time_angle = None
        # Send zero command
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.0
        cmd.vel_right = 0.0
        self.wheels_pub.publish(cmd)
        return EmptyResponse()

    def shutdown_hook(self):
        """Called when node is shutting down - stop the wheels"""
        self.log("Shutting down - stopping wheels")
        
        # Reset PID state
        self.integral_error = 0.0
        self.last_error = 0.0
        self.integral_error_angle = 0.0
        self.last_error_angle = 0.0
        
        # Send zero velocity command
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = 0.0
        cmd.vel_right = 0.0
        self.wheels_pub.publish(cmd)
        
        # Publish zero omega
        omega_msg = Float64()
        omega_msg.data = 0.0
        self.omega_pub.publish(omega_msg)
        
        # Give it a moment to publish
        rospy.sleep(0.1)


def main():
    # create the node
    node = LaneControllerNode(node_name='lane_controller_node')
    # keep spinning
    rospy.spin()


if __name__ == '__main__':
    main()
