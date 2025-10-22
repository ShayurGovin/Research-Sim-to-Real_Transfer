#!/usr/bin/env python3
import rospy, cv2, numpy as np, time, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Point, Twist

def _clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class RedNavRL:
    def __init__(self):
        self.bridge = CvBridge()

        self.image_topic  = rospy.get_param("~image_topic",  "/pepper/camera/front/image_raw")
        self.cmd_topic    = rospy.get_param("~cmd_topic",    "/pepper/cmd_vel")
        self.min_area     = int(rospy.get_param("~min_area", 500))

        self.lower1 = np.array(rospy.get_param("~lower1", [0,  50, 30]),  dtype=np.uint8)
        self.upper1 = np.array(rospy.get_param("~upper1", [12,255,255]), dtype=np.uint8)
        self.lower2 = np.array(rospy.get_param("~lower2", [168,50, 30]), dtype=np.uint8)
        self.upper2 = np.array(rospy.get_param("~upper2", [180,255,255]), dtype=np.uint8)

        self.publish_debug   = bool(rospy.get_param("~publish_debug", True))
        self.control_enabled = bool(rospy.get_param("~control_enabled", True))

        self.k_ang        = float(rospy.get_param("~k_ang", 1.2))
        self.max_ang      = float(rospy.get_param("~max_ang", 0.8))
        self.v_forward    = float(rospy.get_param("~v_forward", 0.8))
        self.align_thresh = float(rospy.get_param("~align_thresh", 0.18))
        self.scan_speed   = float(rospy.get_param("~scan_speed", 0.35))
        self.lost_timeout = float(rospy.get_param("~lost_timeout", 0.6))
        self.loop_hz      = float(rospy.get_param("~loop_hz", 20.0))

        self.bearing_is_normalized = bool(rospy.get_param("~bearing_is_normalized", False))
        self.norm_to_rad_scale     = float(rospy.get_param("~norm_to_rad_scale", math.radians(45.0)))

        self.filter_tau   = float(rospy.get_param("~filter_tau", 0.25))
        self.max_lin_acc  = float(rospy.get_param("~max_lin_acc", 0.35))
        self.max_ang_acc  = float(rospy.get_param("~max_ang_acc", 1.8))
        self.imu_topic    = rospy.get_param("~imu_topic", "")
        self.tip_soft_deg = float(rospy.get_param("~tip_soft_deg", 8.0))
        self.tip_hard_deg = float(rospy.get_param("~tip_hard_deg", 12.0))

        self.fx = None; self.cx = None
        self.last_visible = False
        self.last_bearing = 0.0
        self.last_px = -1; self.last_py = -1
        self.t_last_seen = 0.0

        self.bearing_filt = 0.0
        self.prev_lin = 0.0
        self.prev_ang = 0.0
        self.last_cmd_time = time.time()

        self.roll = 0.0; self.pitch = 0.0
        self.have_imu = False

        self.pub_visible = rospy.Publisher("target_visible", Bool, queue_size=1)
        self.pub_bearing = rospy.Publisher("target_bearing", Float32, queue_size=1)
        self.pub_pixel   = rospy.Publisher("target_pixel", Point, queue_size=1)
        self.pub_debug   = rospy.Publisher("debug_image", Image, queue_size=1)
        self.pub_cmd     = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)

        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2**24)
        cam_info_topic = self.image_topic.replace("image_raw", "camera_info")
        rospy.Subscriber(cam_info_topic, CameraInfo, self.caminfo_cb, queue_size=1)
        if self.imu_topic:
            rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=1)

        rospy.on_shutdown(self.stop_robot)
        rospy.loginfo("RL node: img=%s cmd=%s imu=%s", self.image_topic, self.cmd_topic, self.imu_topic or "(disabled)")

        self.run()

    def caminfo_cb(self, msg):
        if msg.K and msg.K[0] > 0:
            self.fx, self.cx = msg.K[0], msg.K[2]

    def imu_cb(self, msg: Imu):
        q = msg.orientation
        sinr_cosp = 2*(q.w*q.x + q.y*q.z)
        cosr_cosp = 1 - 2*(q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.asin(_clamp(sinp, -1.0, 1.0))
        self.roll  = math.degrees(roll)
        self.pitch = math.degrees(pitch)
        self.have_imu = True

    def _to_bgr8(self, msg):
        enc = (msg.encoding or "").lower()
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if enc == "rgb8":
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == "mono8":
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    def image_cb(self, msg):
        try:
            bgr = self._to_bgr8(msg)
        except Exception as e:
            rospy.logwarn_throttle(2.0, "cv_bridge failed: %s", e)
            return

        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower1, self.upper1) | cv2.inRange(hsv, self.lower2, self.upper2)

        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        visible = False; bearing = 0.0; px = py = -1
        draw_img = bgr.copy() if self.publish_debug else bgr

        if contours:
            i = int(np.argmax([cv2.contourArea(c) for c in contours]))
            x,y,w,h = cv2.boundingRect(contours[i])
            area = w*h
            if area >= self.min_area:
                cx_blob = x + w/2.0; cy_blob = y + h/2.0
                px, py = int(cx_blob), int(cy_blob)
                visible = True
                bearing = float((cx_blob - self.cx)/self.fx) if (self.fx and self.cx is not None) \
                          else float((cx_blob / bgr.shape[1]) * 2.0 - 1.0)
                if self.publish_debug:
                    cv2.rectangle(draw_img, (x,y), (x+w, y+h), (0,255,0), 2)
                    cv2.circle(draw_img, (px,py), 4, (0,0,255), -1)

        self.last_visible  = visible
        self.last_bearing  = bearing
        if visible: self.t_last_seen = time.time()
        self.pub_visible.publish(visible)
        self.pub_bearing.publish(bearing)
        self.pub_pixel.publish(Point(x=px, y=py, z=0.0))

        if self.publish_debug:
            try:
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(draw_img, encoding="bgr8"))
            except Exception: pass

    def run(self):
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            if self.control_enabled:
                cmd = Twist()
                t_since = time.time() - self.t_last_seen
                b = self.last_bearing
                if self.bearing_is_normalized:
                    b = b * self.norm_to_rad_scale

                if self.last_visible and t_since <= self.lost_timeout:
                    ang = - self.k_ang * b
                    ang = max(-self.max_ang, min(self.max_ang, ang))
                    cmd.angular.z = ang
                    cmd.linear.x = self.v_forward if abs(b) < self.align_thresh else 0.0
                else:
                    cmd.angular.z = self.scan_speed
                    cmd.linear.x  = 0.0

                now = time.time()
                dt  = max(1.0/self.loop_hz, now - self.last_cmd_time)
                self.last_cmd_time = now

                b_use = self.last_bearing
                if self.bearing_is_normalized:
                    b_use *= self.norm_to_rad_scale
                if self.filter_tau > 0.0:
                    alpha = _clamp(dt / (self.filter_tau + dt), 0.0, 1.0)
                    self.bearing_filt = (1.0 - alpha)*self.bearing_filt + alpha*b_use
                    b_use = self.bearing_filt

                tilt = max(abs(self.roll), abs(self.pitch)) if self.have_imu else 0.0
                tip_scale = 1.0
                if self.imu_topic:
                    if tilt >= self.tip_hard_deg: tip_scale = 0.0
                    elif tilt >= self.tip_soft_deg:
                        span = max(1e-6, self.tip_hard_deg - self.tip_soft_deg)
                        tip_scale = _clamp(1.0 - (tilt - self.tip_soft_deg)/span, 0.0, 1.0)

                cmd.linear.x  *= tip_scale
                cmd.angular.z *= (0.5 + 0.5*tip_scale)

                max_dlin = self.max_lin_acc * dt
                max_dang = self.max_ang_acc * dt
                lin_out  = self.prev_lin + _clamp(cmd.linear.x - self.prev_lin, -max_dlin, max_dlin)
                ang_out  = self.prev_ang + _clamp(cmd.angular.z - self.prev_ang, -max_dang, max_dang)

                lin_out = _clamp(lin_out, -0.4, 0.4)
                ang_out = _clamp(ang_out, -self.max_ang, self.max_ang)

                cmd.linear.x, cmd.angular.z = lin_out, ang_out
                self.prev_lin, self.prev_ang = lin_out, ang_out

                rospy.loginfo_throttle(0.5, "CMD lin=%.2f ang=%.2f vis=%s tilt=%.1f°",
                                       cmd.linear.x, cmd.angular.z,
                                       str(self.last_visible), tilt)

                self.pub_cmd.publish(cmd)
            rate.sleep()

    def stop_robot(self):
        try: self.pub_cmd.publish(Twist())
        except Exception: pass

if __name__ == "__main__":
    rospy.init_node("RL")
    RedNavRL()
