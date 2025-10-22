#!/usr/bin/env python3
import math, time, threading, random
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from collections import deque

import rospy
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState, ModelStates
import cv2


def _clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class _RosFrontend:
    """Improved ROS Frontend with atomic observations and better state management"""
    def __init__(self,
                 image_topic="/pepper/camera/front/image_raw",
                 caminfo_topic="/pepper/camera/front/camera_info",
                 scan_topics=("/pepper/scan_front", "/pepper/hokuyo_scan"),
                 cmd_topic="/pepper/cmd_vel",
                 lower1=(0, 150, 80), upper1=(10, 255, 200),
                 lower2=(170, 150, 80), upper2=(180, 255, 200),
                 min_area=200,
                 debug=True):
        self.lock = threading.RLock()
        self.bridge = CvBridge()

        self.fx = None
        self.cx = None

        self.obs_data = {
            'timestamp': 0.0,
            'visible': False,
            'bearing': 0.0,
            'area': None,
            'scan': None,
            'img_width': None
        }

        self.bearing_history = deque(maxlen=3)
        self.area_history = deque(maxlen=5)

        self.lower1 = np.array(lower1, dtype=np.uint8)
        self.upper1 = np.array(upper1, dtype=np.uint8)
        self.lower2 = np.array(lower2, dtype=np.uint8)
        self.upper2 = np.array(upper2, dtype=np.uint8)
        self.min_area = int(min_area)
        self.debug = bool(debug)

        self.pub_cmd = rospy.Publisher(cmd_topic, Twist, queue_size=1)

        rospy.Subscriber(image_topic, Image, self._img_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber(caminfo_topic, CameraInfo, self._caminfo_cb, queue_size=1)

        chosen_scan = scan_topics[0] if scan_topics else "/pepper/scan_front"
        rospy.Subscriber(chosen_scan, LaserScan, self._scan_cb, queue_size=1)

        self._last_valid_obs_time = 0.0
        self._obs_timeout = 1.0

    def _caminfo_cb(self, msg: CameraInfo):
        if msg.K and msg.K[0] > 0:
            with self.lock:
                self.fx, self.cx = float(msg.K[0]), float(msg.K[2])

    def _img_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        lower1 = np.array([0, 150, 80], dtype=np.uint8)
        upper1 = np.array([10, 255, 200], dtype=np.uint8)
        lower2 = np.array([170, 150, 80], dtype=np.uint8)
        upper2 = np.array([180, 255, 200], dtype=np.uint8)
        
        mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        
        valid_contours = []
        min_aspect_ratio = 1.5  
        min_area_for_detection = 75

        if self.debug:
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area_for_detection:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            if w == 0: continue
            
            aspect_ratio = float(h) / w
            
            if self.debug:
                rospy.loginfo(f"Contour found -> Area: {int(area)}, Aspect Ratio: {aspect_ratio:.2f}")

            if aspect_ratio < min_aspect_ratio:
                continue
            
            valid_contours.append(cnt)

        visible = False; bearing = 0.0; area = None
        
        if valid_contours:
            largest_valid_contour = max(valid_contours, key=cv2.contourArea)
            
            x, y, w, h = cv2.boundingRect(largest_valid_contour)

            if self.debug:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(img, "TARGET", (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            area = w * h; cx_blob = x + w / 2.0; img_width = img.shape[1]
            if self.fx and self.cx is not None:
                bearing = float((cx_blob - self.cx) / self.fx)
            else:
                norm = float((cx_blob / img_width) * 2.0 - 1.0)
                bearing = norm * (math.pi / 3.0)
            visible = True
            self.bearing_history.append(bearing)
            self.area_history.append(area)

        if self.debug:
            pass

        with self.lock:
            self.obs_data.update({
                'timestamp': time.time(), 'visible': visible, 'bearing': float(np.clip(bearing, -math.pi, math.pi)),
                'area': area, 'img_width': img.shape[1]
            })
            self._last_valid_obs_time = self.obs_data['timestamp']

    def _scan_cb(self, msg: LaserScan):
        with self.lock:
            self.obs_data['scan'] = msg

    def get_stable_bearing(self):
        if len(self.bearing_history) == 0:
            return 0.0
        return float(np.median(list(self.bearing_history)))

    def get_stable_area(self):
        if len(self.area_history) == 0:
            return None
        return float(np.median(list(self.area_history)))

    def get_front_min_range(self, fov_deg=40.0, clip=(0.0, 5.0)):
        with self.lock:
            scan = self.obs_data['scan']
            if scan is None:
                return None
            half_fov = math.radians(fov_deg / 2.0)
            angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
            mask = (angles >= -half_fov) & (angles <= +half_fov)
            if not np.any(mask):
                return None
            ranges = np.asarray(scan.ranges, dtype=np.float32)[mask]
            valid_ranges = ranges[(ranges >= scan.range_min) &
                                  (ranges <= scan.range_max) &
                                  np.isfinite(ranges)]
            if valid_ranges.size == 0:
                return None
            return float(np.clip(np.min(valid_ranges), clip[0], clip[1]))

    def get_atomic_observation(self):
        with self.lock:
            current_time = time.time()
            if (current_time - self._last_valid_obs_time) > self._obs_timeout:
                if self.debug:
                    rospy.logwarn("Observation data is stale (%.2fs old)",
                                  current_time - self._last_valid_obs_time)
                return None, None, None, False
            vis = 1.0 if self.obs_data['visible'] else 0.0
            bearing = self.get_stable_bearing() if self.obs_data['visible'] else 0.0
            area = self.get_stable_area() if self.obs_data['visible'] else None
            return vis, bearing, area, True

    def send_cmd(self, v, w):
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def stop(self):
        try: self.send_cmd(0.0, 0.0)
        except Exception: pass


class ImprovedPepperEnv(gym.Env):
    """Improved Pepper Environment with spawning support"""
    metadata = {"render_modes": []}

    def __init__(self,
                 image_topic="/pepper/camera/front/image_raw",
                 caminfo_topic="/pepper/camera/front/camera_info",
                 scan_topic="/pepper/scan_front",
                 cmd_topic="/pepper/cmd_vel",
                 control_hz=20.0,
                 max_episode_steps=600,
                 lin_max=0.35, ang_max=1.0,
                 front_fov_deg=50.0, range_clip=5.0, collision_thresh=0.30,
                 success_bearing_rad=math.radians(10.0),
                 success_dwell_steps=8,
                 success_distance_thresh=0.5,
                 close_distance_thresh=0.4,
                 r_step=-0.005,
                 r_visible=0.05,
                 r_alignment_scale=0.4,
                 alignment_sharpness=15.0,
                 r_exploration=0.01,
                 r_collision=-2.0,
                 r_success=5.0,
                 r_proximity_scale=0.5,
                 r_backward_penalty=0.02,
                 lower1=(0, 50, 30), upper1=(12, 255, 255),
                 lower2=(168, 50, 30), upper2=(180, 255, 255),
                 min_area=200,
                 model_name="pepper",
                 enable_random_spawn=True,
                 spawn_positions=None,
                 yaw_range=(-math.pi, math.pi),
                 debug=False,
                 seed=0):
        super().__init__()
        if not rospy.core.is_initialized():
            rospy.init_node("improved_pepper_env", anonymous=True, disable_signals=True)

        self.rng = np.random.RandomState(seed)
        self.dt = 1.0 / float(control_hz)
        self.max_steps = int(max_episode_steps)

        self.lin_max = float(lin_max); self.ang_max = float(ang_max)
        self.front_fov_deg = float(front_fov_deg)
        self.range_clip = float(range_clip)
        self.collision_thresh = float(collision_thresh)

        self.success_bearing = float(success_bearing_rad)
        self.success_dwell_steps = int(success_dwell_steps)
        self.success_distance_thresh = float(success_distance_thresh)
        self.close_distance_thresh = float(close_distance_thresh)

        self.r_step = float(r_step)
        self.r_visible = float(r_visible)
        self.r_alignment_scale = float(r_alignment_scale)
        self.alignment_sharpness = math.radians(float(alignment_sharpness))
        self.r_exploration = float(r_exploration)
        self.r_collision = float(r_collision)
        self.r_success = float(r_success)
        self.r_backward_penalty = float(r_backward_penalty)
        self.r_proximity_scale = float(r_proximity_scale)


        self.model_name_cfg = str(model_name) if model_name else None
        self.enable_random_spawn = bool(enable_random_spawn)
        self.yaw_min, self.yaw_max = float(yaw_range[0]), float(yaw_range[1])

        if spawn_positions is None:
            self.spawn_positions = [(-2, -1.7), (-1.3, -4.597), (-3.0, 0.4), (0, -1)]
        else:
            norm = []
            for p in spawn_positions:
                if isinstance(p, dict):
                    norm.append((float(p.get("x", 0.0)), float(p.get("y", 0.0))))
                else:
                    x, y = p[:2]
                    norm.append((float(x), float(y)))
            self.spawn_positions = norm

        self._gazebo_model_name = None
        self._set_state_srv = None

        self.io = _RosFrontend(
            image_topic=image_topic,
            caminfo_topic=caminfo_topic,
            scan_topics=(scan_topic, "/pepper/hokuyo_scan"),
            cmd_topic=cmd_topic,
            lower1=lower1, upper1=upper1, lower2=lower2, upper2=upper2,
            min_area=min_area, debug=debug
        )

        self.observation_space = spaces.Box(
            low=np.array([0.0, -1.0, 0.0, 0.0], dtype=np.float32),
            high=np.array([1.0,  1.0, 1.0, 1.0], dtype=np.float32),
            dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=np.array([-self.lin_max, -self.ang_max], dtype=np.float32),
            high=np.array([ self.lin_max,  self.ang_max], dtype=np.float32),
            dtype=np.float32
        )

        self.reset_episode_state()
        self.debug = bool(debug)

    def _ensure_gazebo_clients(self):
        if self._gazebo_model_name is None:
            msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5.0)
            if self.model_name_cfg and self.model_name_cfg in msg.name:
                self._gazebo_model_name = self.model_name_cfg
            else:
                for name in msg.name:
                    if "pepper" in name.lower():
                        self._gazebo_model_name = name
                        break
            if self._gazebo_model_name is None:
                raise RuntimeError(f"No Gazebo model matching 'pepper' found. Models: {list(msg.name)}")
        if self._set_state_srv is None:
            rospy.wait_for_service("/gazebo/set_model_state")
            self._set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    @staticmethod
    def _yaw_to_quat(yaw):
        return math.sin(yaw/2.0), math.cos(yaw/2.0)

    def _teleport_to_random_fixed(self):
        self._ensure_gazebo_clients()
        x, y = random.choice(self.spawn_positions)
        yaw = random.uniform(self.yaw_min, self.yaw_max)
        qz, qw = self._yaw_to_quat(yaw)

        ms = ModelState()
        ms.model_name = self._gazebo_model_name
        ms.pose.position.x = float(x); ms.pose.position.y = float(y); ms.pose.position.z = 0.0
        ms.pose.orientation.z = qz; ms.pose.orientation.w = qw
        ms.twist = Twist()

        resp = self._set_state_srv(ms)
        if self.debug:
            rospy.loginfo(f"[Spawn] {ms.model_name} -> (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}) success={resp.success}")
        rospy.sleep(0.25)

    def reset_episode_state(self):
        self.step_count = 0; self.success_dwell = 0; self.prev_range = None
        self.prev_area = None; self.last_angular_vel = 0.0; self.cumulative_reward = 0.0

    def _get_observation(self):
        vis, bearing, area, data_valid = self.io.get_atomic_observation()
        if not data_valid:
            return np.array([0.0, 0.0, 1.0, 0.0], dtype=np.float32), None
        range_reading = self.io.get_front_min_range(self.front_fov_deg, (0.0, self.range_clip))
        if range_reading is None or not np.isfinite(range_reading):
            range_reading = self.range_clip
        bearing_normalized = np.clip(bearing / math.pi, -1.0, 1.0)
        range_normalized = np.clip(range_reading / self.range_clip, 0.0, 1.0)
        bearing_confidence = min(len(self.io.bearing_history) / 3.0, 1.0) if vis > 0.5 else 0.0
        obs = np.array([vis, bearing_normalized, range_normalized, bearing_confidence], dtype=np.float32)
        return obs, area

    def _calculate_reward(self, obs, area_now, linear_vel):
        vis, bearing_norm, range_norm, confidence = obs
        bearing = bearing_norm * math.pi
        range_reading = range_norm * self.range_clip

        reward = self.r_step
        
        if linear_vel < -0.01:
            reward -= self.r_backward_penalty

        if vis >= 0.5:
            reward += self.r_visible
            alignment = math.exp(-(bearing**2) / (2 * self.alignment_sharpness**2))
            reward += self.r_alignment_scale * alignment * confidence
            proximity_reward = (1.0 / (range_reading + 0.2)) * self.r_proximity_scale
            reward += proximity_reward * alignment
            
        else:
            exploration = abs(self.last_angular_vel) / self.ang_max
            reward += self.r_exploration * exploration

        if area_now is not None and self.prev_area is not None:
            area_ratio = area_now / max(self.prev_area, 1.0)
            if area_ratio > 1.1:
                reward += 0.02 * (area_ratio - 1.0)

        self.prev_area = area_now
        return reward

    def _check_termination(self, obs):
        vis, bearing_norm, range_norm, confidence = obs
        range_reading = range_norm * self.range_clip

        if range_reading <= self.collision_thresh:
            return True, False, {"termination_reason": "collision"}

        if (vis >= 0.5 and
            abs(bearing_norm * math.pi) <= self.success_bearing and
            range_reading <= self.success_distance_thresh and
            confidence >= 0.5):
            self.success_dwell += 1
        else:
            self.success_dwell = 0

        if self.success_dwell >= self.success_dwell_steps:
            return True, True, {"termination_reason": "success", "success_dwell": self.success_dwell}

        return False, False, {}

    def reset(self, *, seed=None, options=None):
        if seed is not None: self.rng.seed(seed)
        self.reset_episode_state()
        self.io.stop(); rospy.sleep(0.3)
        if self.enable_random_spawn:
            try: self._teleport_to_random_fixed()
            except Exception as e: rospy.logwarn(f"[Spawn] teleport failed: {e}")

        obs, area = self._get_observation()
        self.prev_area = area
        return obs, {}

    def step(self, action):
        self.step_count += 1
        v = float(np.clip(action[0], -self.lin_max, self.lin_max))
        w = float(np.clip(action[1], -self.ang_max, self.ang_max))
        self.last_angular_vel = w

        self.io.send_cmd(v, w)
        rospy.sleep(self.dt)

        obs, area = self._get_observation()
        reward = self._calculate_reward(obs, area, v)
        self.cumulative_reward += reward
        
        terminated, success, term_info = self._check_termination(obs)
        truncated = (self.step_count >= self.max_steps)

        if terminated:
            if success:
                reward += self.r_success
            else:
                reward += self.r_collision

        info = { "success": success, "episode_step": self.step_count,
                 "cumulative_reward": self.cumulative_reward, **term_info }

        if self.debug and (terminated or truncated):
            rospy.loginfo("Episode ended: %s (total_reward=%.2f)",
                          info.get("termination_reason", "truncated"), self.cumulative_reward)
        
        if terminated or truncated:
            self.io.stop()

        return obs, reward, terminated, truncated, info

    def render(self): return None
    def close(self): self.io.stop()


def make_improved_env(**env_kwargs):
    def _thunk():
        return ImprovedPepperEnv(**env_kwargs)
    return _thunk