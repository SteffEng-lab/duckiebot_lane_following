#!/usr/bin/env python3
# SDTS Node - Scenario Driven Transition System (Duckietown Compatible)
# Supervises the LaneControllerNode according to Section 5 of the paper.

import os
import rospy
from duckietown.dtros import DTROS, NodeType

from geometry_msgs.msg import Point
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


class SDTSNode(DTROS):

    def __init__(self, node_name="sdts_node"):
        super(SDTSNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # Vehicle name
        self._veh = os.environ.get("VEHICLE_NAME", "deutschbot")

        # === SDTS STATES ===
        self.state = "Idle"
        self.state_enter_time = rospy.Time.now()

        # === PARAMETERS ===
        # Use controller defaults
        self.v_nominal = rospy.get_param("~v_nominal", 0.005)
        self.v_turn = rospy.get_param("~v_turn", 0.002)

        # SDTS thresholds
        self.x_lost_thresh = rospy.get_param("~x_lost_thresh", 0.5)
        self.t_lost_timeout = rospy.get_param("~t_lost_timeout", 1.0)
        self.t_recover_timeout = rospy.get_param("~t_recover_timeout", 4.0)
        self.recenter_thresh = rospy.get_param("~recenter_thresh", 0.05)
        self.t_stable = rospy.get_param("~t_stable", 0.5)

        # === OBSERVED FEATURES ===
        self.x_v = None
        self.x_m = None
        self.corner = False
        self.corner_dir = "none"

        self.last_seen = rospy.Time.now()
        self.lines_visible = "none"
        
        # === PUBLISHERS ===
        # Recovery wheel commands
        self.wheels_pub = rospy.Publisher(
            f"/{self._veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=1
        )

        # === SUBSCRIBERS ===
        rospy.Subscriber(f"/{self._veh}/lane_following/vanishing_point", Point, self.cb_vanish)
        rospy.Subscriber(f"/{self._veh}/lane_following/mid_point", Point, self.cb_mid)
        rospy.Subscriber(f"/{self._veh}/lane_following/corner_detected", Bool, self.cb_corner)
        rospy.Subscriber(f"/{self._veh}/lane_following/corner_direction", String, self.cb_corner_dir)



        # State debug output
        self.state_pub = rospy.Publisher(
            f"/{self._veh}/sdts/state",
            String,
            queue_size=1
        )

        # === CONTROLLER SERVICES ===
        self.reset_srv = self._wait_for_srv("reset")
        self.enable_srv = self._wait_for_srv("enable")
        self.disable_srv = self._wait_for_srv("disable")

        # Timer loop
        self.timer = rospy.Timer(rospy.Duration(0.05), self.tick)

        self.loginfo("SDTS Node initialized (state=Idle).")


    # ----------------------------------------------------------------------
    # Helper: Wait for lane controller service
    def _wait_for_srv(self, srv_name):
        full = f"/{self._veh}/lane_controller/{srv_name}"
        try:
            rospy.wait_for_service(full, timeout=5.0)
            return rospy.ServiceProxy(full, Empty)
        except Exception as e:
            self.logwarn(f"Could not connect to {full}: {e}")
            return None


    # ----------------------------------------------------------------------
    # CALLBACKS
    def cb_vanish(self, msg):
        self.x_v = msg.x
        self.last_seen = rospy.Time.now()
        if self.x_m is not None:
            self.lines_visible = "both"
        else:
            self.lines_visible = "one"

    def cb_mid(self, msg):
        self.x_m = msg.x
        self.last_seen = rospy.Time.now()
        if self.x_v is not None:
            self.lines_visible = "both"

    def cb_corner(self, msg):
        self.corner = bool(msg.data)

    def cb_corner_dir(self, msg):
        self.corner_dir = msg.data


    # ----------------------------------------------------------------------
    # Recovery motion: slow rotation to reacquire line
    def recovery_motion(self):
        cmd = WheelsCmdStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.vel_left = -0.5
        cmd.vel_right = 0.5
        self.wheels_pub.publish(cmd)


    # ----------------------------------------------------------------------
    # State transition helper
    def switch_state(self, new_state):
        self.state = new_state
        self.state_enter_time = rospy.Time.now()
        self.loginfo(f"SDTS -> {new_state}")
        self.state_pub.publish(new_state)


    # ----------------------------------------------------------------------
    # SERVICE WRAPPERS
    def call_reset(self):
        try:
            if self.reset_srv:
                self.reset_srv()
        except:
            pass

    def call_enable(self):
        try:
            if self.enable_srv:
                self.enable_srv()
        except:
            pass

    def call_disable(self):
        try:
            if self.disable_srv:
                self.disable_srv()
        except:
            pass


    # ----------------------------------------------------------------------
    # MAIN SDTS LOGIC
    def tick(self, _event):
        now = rospy.Time.now()

        # Lost-line predicate
        lost = (now - self.last_seen).to_sec() > self.t_lost_timeout

        # x-abs for thresholding
        x_abs = abs(self.x_v) if self.x_v is not None else float("inf")


        # ----------------------
        # STATE: IDLE
        # ----------------------
        if self.state == "Idle":
            # Auto-enable logic
            self.call_enable()
            rospy.set_param(f"/{self._veh}/lane_controller/v", self.v_nominal)
            self.call_reset()
            self.switch_state("LaneFollowing")
            return


        # ----------------------
        # STATE: LANE FOLLOWING
        # ----------------------
        if self.state == "LaneFollowing":

            if self.corner:
                rospy.set_param(f"/{self._veh}/lane_controller/v", self.v_turn)
                self.call_reset()
                self.switch_state("CornerManeuver")
                return

            if lost or x_abs > self.x_lost_thresh:
                self.call_disable()
                self.switch_state("Recovery")
                return


        # ----------------------
        # STATE: CORNER MANEUVER
        # ----------------------
        if self.state == "CornerManeuver":

            # Exit if corner disappears and lines stable
            if (not self.corner) and (self.lines_visible == "both") and (x_abs < self.recenter_thresh):
                if (now - self.state_enter_time).to_sec() >= self.t_stable:
                    rospy.set_param(f"/{self._veh}/lane_controller/v", self.v_nominal)
                    self.call_reset()
                    self.call_enable()
                    self.switch_state("LaneFollowing")
                    return

            # Corner lost for too long
            if lost and (now - self.state_enter_time).to_sec() > self.t_recover_timeout:
                self.call_disable()
                self.switch_state("Recovery")
                return


        # ----------------------
        # STATE: RECOVERY
        # ----------------------
        if self.state == "Recovery":

            # If lines reliably reappear → return to lane following
            if self.lines_visible == "both" and (x_abs < self.recenter_thresh):
                if (now - self.last_seen).to_sec() < self.t_lost_timeout:
                    rospy.set_param(f"/{self._veh}/lane_controller/v", self.v_nominal)
                    self.call_reset()
                    self.call_enable()
                    self.switch_state("LaneFollowing")
                    return

            # Otherwise run recovery wheel motion
            self.recovery_motion()


        # Disabled and Stopped states (if you ever add manual overrides)
        # would go here.


# ----------------------------------------------------------------------
def main():
    node = SDTSNode()
    rospy.spin()


if __name__ == "__main__":
    main()
