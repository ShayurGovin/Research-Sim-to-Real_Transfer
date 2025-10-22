#!/usr/bin/env python3
import os, sys, time, math, signal
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import cv2

class PathToImage:
    def __init__(self):
        self.odom_topic   = rospy.get_param("~odom_topic", "/pepper/odom")
        self.kill_topic   = rospy.get_param("~kill_topic", "/pepper/kill")
        self.reached_topic= rospy.get_param("~reached_topic", "/target_reached")
        self.img_size     = int(rospy.get_param("~img_size", 1024))
        self.margin_px    = int(rospy.get_param("~margin_px", 40))
        self.bg_color     = tuple(rospy.get_param("~bg_color", [255,255,255]))
        self.path_color   = tuple(rospy.get_param("~path_color", [20,20,20]))
        self.start_color  = tuple(rospy.get_param("~start_color",[40,180,40]))
        self.end_color    = tuple(rospy.get_param("~end_color",  [40,40,200]))
        self.grid_color   = tuple(rospy.get_param("~grid_color", [220,220,220]))
        self.line_thick   = int(rospy.get_param("~line_thick", 3))
        self.dot_r        = int(rospy.get_param("~dot_r", 6))
        self.downsample_n = int(rospy.get_param("~downsample_n", 1))
        self.save_dir     = os.path.expanduser(rospy.get_param("~save_dir", "~/Sim Evaluation"))
        self.save_prefix  = rospy.get_param("~save_prefix", "path")
        self.save_csv     = bool(rospy.get_param("~save_csv", True))
        self.add_grid     = bool(rospy.get_param("~add_grid", True))
        self.grid_ticks   = int(rospy.get_param("~grid_ticks", 8))
        self.annotate     = bool(rospy.get_param("~annotate", True))
        self.scale_pad    = float(rospy.get_param("~scale_pad", 0.05))

        os.makedirs(self.save_dir, exist_ok=True)

        self.xy = []
        self._kill = False
        self._t0 = rospy.Time.now().to_sec()

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber(self.kill_topic, Bool, self.kill_cb, queue_size=1)
        rospy.Subscriber(self.reached_topic, Bool, self.reached_cb, queue_size=1)

        rospy.on_shutdown(self.on_shutdown)
        signal.signal(signal.SIGTERM, self._sig_kill)
        signal.signal(signal.SIGINT,  self._sig_kill)

        rospy.loginfo("PathToImage: listening to %s; will save PNG/CSV to %s",
                      self.odom_topic, self.save_dir)

    def _sig_kill(self, *_):
        self._kill = True

    def kill_cb(self, msg: Bool):
        if msg.data:
            self._kill = True

    def reached_cb(self, msg: Bool):
        if msg.data:
            self._kill = True

    def odom_cb(self, msg: Odometry):
        p: Pose = msg.pose.pose
        self.xy.append((p.position.x, p.position.y))
        if self._kill:
            rospy.signal_shutdown("stop requested")

    def _world_to_img(self, pts_xy):
        pts = np.array(pts_xy, dtype=np.float32)
        xmin, ymin = pts.min(axis=0)
        xmax, ymax = pts.max(axis=0)
        dx = xmax - xmin
        dy = ymax - ymin
        if dx == 0 and dy == 0:
            dx = dy = 1e-2
        pad_x = max(dx, 1e-6) * self.scale_pad
        pad_y = max(dy, 1e-6) * self.scale_pad
        xmin -= pad_x; xmax += pad_x
        ymin -= pad_y; ymax += pad_y
        dx = xmax - xmin; dy = ymax - ymin

        span = max(dx, dy)
        usable = self.img_size - 2*self.margin_px
        s = usable / span

        offx = (self.img_size - (dx * s)) * 0.5
        offy = (self.img_size - (dy * s)) * 0.5

        img_pts = []
        for x,y in pts_xy:
            ix = int((x - xmin)*s + offx)
            iy = int(self.img_size - ((y - ymin)*s + offy))
            img_pts.append((ix, iy))
        return img_pts, s, (xmin, ymin, xmax, ymax)

    def _draw_grid(self, img, s, extents):
        if not self.add_grid or self.grid_ticks < 2: return
        xmin, ymin, xmax, ymax = extents
        step = 1.0 / (self.grid_ticks - 1)
        for i in range(self.grid_ticks):
            tx = xmin + (xmax - xmin) * (i*step)
            ty = ymin + (ymax - ymin) * (i*step)
            p1,_,_ = self._world_to_img([(tx, ymin)])
            p2,_,_ = self._world_to_img([(tx, ymax)])
            cv2.line(img, p1[0], p2[0], self.grid_color, 1, cv2.LINE_AA)
            p3,_,_ = self._world_to_img([(xmin, ty)])
            p4,_,_ = self._world_to_img([(xmax, ty)])
            cv2.line(img, p3[0], p4[0], self.grid_color, 1, cv2.LINE_AA)

    def _annotate_scale(self, img, s):
        if not self.annotate: return
        bar_m = 1.0
        bar_px = int(bar_m * s)
        x0 = 20; y0 = self.img_size - 20
        cv2.line(img, (x0, y0), (x0+bar_px, y0), (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img, "1 m", (x0+bar_px+10, y0+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 2, cv2.LINE_AA)

    def save_outputs(self):
        if len(self.xy) < 2:
            rospy.logwarn("No path points recorded; nothing to save.")
            return

        pts = self.xy[::max(1,self.downsample_n)]

        img = np.full((self.img_size, self.img_size, 3), self.bg_color, dtype=np.uint8)
        img_pts, s, extents = self._world_to_img(pts)

        self._draw_grid(img, s, extents)
        self._annotate_scale(img, s)

        cv2.polylines(img, [np.array(img_pts, dtype=np.int32)], False, self.path_color, self.line_thick, cv2.LINE_AA)

        cv2.circle(img, img_pts[0],  self.dot_r, self.start_color, -1, cv2.LINE_AA)
        cv2.circle(img, img_pts[-1], self.dot_r, self.end_color,   -1, cv2.LINE_AA)
        cv2.putText(img, "START", (img_pts[0][0]+8, img_pts[0][1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.start_color, 2, cv2.LINE_AA)
        cv2.putText(img, "END", (img_pts[-1][0]+8, img_pts[-1][1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.end_color, 2, cv2.LINE_AA)

        ts = time.strftime("%Y%m%d_%H%M%S")
        png = os.path.join(self.save_dir, f"{self.save_prefix}_{ts}.png")
        csv = os.path.join(self.save_dir, f"{self.save_prefix}_{ts}.csv")

        cv2.imwrite(png, img)
        if self.save_csv:
            np.savetxt(csv, np.array(self.xy), delimiter=",", header="x,y", comments="")

        rospy.loginfo("Saved path image: %s", png)
        if self.save_csv:
            rospy.loginfo("Saved path CSV:   %s", csv)

    def on_shutdown(self):
        try:
            self.save_outputs()
        except Exception as e:
            rospy.logerr("Failed to save path outputs: %s", e)

def main():
    rospy.init_node("pepper_path_to_image")
    PathToImage()
    rospy.spin()

if __name__ == "__main__":
    main()
