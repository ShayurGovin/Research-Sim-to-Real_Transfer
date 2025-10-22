#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import sys, select, termios, tty

class BridgeProbe:
    def __init__(self):
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/pepper/cmd_vel")
        self.camera_topic  = rospy.get_param("~camera_topic",  "/pepper/camera/front/image_raw")
        self.odom_topic    = rospy.get_param("~odom_topic",    "/pepper/odom")
        self.duration_sec  = float(rospy.get_param("~duration_sec", 2.0))
        self.speed_mps     = float(rospy.get_param("~speed_mps", 0.05))

        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.cam_count = 0
        self.odom_count = 0
        self.killed = False

        rospy.Subscriber(self.camera_topic, Image, self._cam_cb, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self._odom_cb, queue_size=1)

        rospy.loginfo("BridgeProbe starting")
        rospy.loginfo(f"  Publishing  : {self.cmd_vel_topic}")
        rospy.loginfo(f"  Subscribing : {self.camera_topic}, {self.odom_topic}")
        rospy.sleep(0.5)

    def _cam_cb(self, _):
        self.cam_count += 1

    def _odom_cb(self, _):
        self.odom_count += 1

    def _check_keyboard(self):
        """Non-blocking keyboard check. Press 'q' to stop."""
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            key = sys.stdin.read(1)
            if key.lower() == 'q':
                rospy.logwarn("🚨 Kill switch pressed (q)! Stopping Pepper immediately.")
                self.killed = True
                self.pub.publish(Twist())

    def forward_and_check(self):
        rate = rospy.Rate(20)
        start = rospy.Time.now()
        twist = Twist()
        twist.linear.x = self.speed_mps

        rospy.loginfo(f"Sending forward cmd: {self.speed_mps:.3f} m/s for {self.duration_sec:.1f}s")
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < self.duration_sec:
            self._check_keyboard()
            if self.killed:
                rospy.loginfo("Aborting motion due to kill switch.")
                break
            self.pub.publish(twist)
            rate.sleep()

        self.pub.publish(Twist())
        rospy.loginfo("Stopped (sent zero Twist).")
        rospy.sleep(0.5)

        rospy.loginfo(f"Camera msgs received: {self.cam_count}")
        rospy.loginfo(f"Odom   msgs received: {self.odom_count}")

        ok_cam  = self.cam_count  > 0
        ok_odom = self.odom_count > 0

        if ok_cam and ok_odom:
            rospy.loginfo("✅ Bridge looks good: sensors flowing and cmd_vel relayed.")
        elif ok_cam and not ok_odom:
            rospy.logwarn("⚠️ Camera OK but no odom — check /naoqi_driver/odom relay.")
        elif ok_odom and not ok_cam:
            rospy.logwarn("⚠️ Odom OK but no camera — check front image relay.")
        else:
            rospy.logerr("❌ No camera or odom received — bridge not working.")

def main():
    rospy.init_node("pepper_forward_test")

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    try:
        bp = BridgeProbe()
        bp.forward_and_check()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == "__main__":
    main()
