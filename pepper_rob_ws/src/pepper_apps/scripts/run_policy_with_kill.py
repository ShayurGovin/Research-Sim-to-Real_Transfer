#!/usr/bin/env python3
import os, sys, argparse, select, termios, tty, signal, time
import rospy
import numpy as np
from stable_baselines3 import PPO
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

sys.path.append('/home/shaytrix2/workspace/pepper_sim_ws/src')
from gym_pepper_env.gym_env.pepper_env import ImprovedPepperEnv

KILL = False

def set_kill(reason="unknown"):
    global KILL
    if not KILL:
        rospy.logwarn(f"[KILL] Triggered ({reason}). Stopping Pepper…")
    KILL = True

def kill_cb(msg: Bool):
    if msg.data:
        set_kill("topic:/pepper/kill")

def send_zero_twist(pub, times=3, sleep_s=0.05):
    z = Twist()
    for _ in range(times):
        pub.publish(z)
        rospy.sleep(sleep_s)

def on_shutdown(stop_pub):
    send_zero_twist(stop_pub, times=5, sleep_s=0.05)
    rospy.logwarn("[KILL] Zero velocity sent on shutdown.")

def make_tty_raw(fd):
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    return old

def restore_tty(fd, old):
    try:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    except Exception:
        pass

def read_q_pressed_nonblocking():
    if not sys.stdin.isatty():
        return False
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        ch = sys.stdin.read(1)
        return ch in ('q', 'Q')
    return False

def main():
    ap = argparse.ArgumentParser("Run trained PPO policy on Pepper with kill switch")
    ap.add_argument("--model", default="/home/shaytrix2/workspace/pepper_sim_ws/ckpts/Policy5/Final5.zip")
    ap.add_argument("--no-reset", action="store_true",
                    help="Do not call env.reset() when episode ends (prevents any respawn in eval).")
    ap.add_argument("--deterministic", action="store_true")

    
    ap.add_argument("--image_topic", default="/naoqi_driver/camera/front/image_raw")


    ap.add_argument("--caminfo_topic", default="/naoqi_driver/camera/front/camera_info")


    ap.add_argument("--scan_topic", default="/naoqi_driver/laser")
    ap.add_argument("--cmd_topic", default="/cmd_vel")
    ap.add_argument("--kill_topic", default="/pepper/kill",
                    help="Subscribe here for std_msgs/Bool to trigger kill (True => stop).")
    args = ap.parse_args()

    if not rospy.core.is_initialized():
        rospy.init_node("pepper_policy_runner", anonymous=True)

    stop_pub = rospy.Publisher(args.cmd_topic, Twist, queue_size=1)
    rospy.Subscriber(args.kill_topic, Bool, kill_cb, queue_size=1)

    rospy.on_shutdown(lambda: on_shutdown(stop_pub))
    signal.signal(signal.SIGTERM, lambda *_: set_kill("SIGTERM"))
    signal.signal(signal.SIGINT,  lambda *_: set_kill("SIGINT"))

    env = ImprovedPepperEnv(
        image_topic=args.image_topic,
        caminfo_topic=args.caminfo_topic,
        scan_topic=args.scan_topic,
        cmd_topic=args.cmd_topic,
        enable_random_spawn=False,
    )

    model = PPO.load(args.model, env=env, device="cpu")

    obs, _ = env.reset()
    rate = rospy.Rate(20)

    old_tty = None
    if sys.stdin.isatty():
        old_tty = make_tty_raw(sys.stdin.fileno())
        rospy.loginfo("Press 'q' to KILL (this terminal must stay focused).")

    global KILL
    try:
        while not rospy.is_shutdown() and not KILL:
            if read_q_pressed_nonblocking():
                set_kill("keyboard:q")
                break

            action, _ = model.predict(obs, deterministic=args.deterministic)
            obs, reward, done, truncated, info = env.step(action)

            if not args.no_reset and (done or truncated):
                obs, _ = env.reset()

            rate.sleep()
    finally:
        if old_tty is not None:
            restore_tty(sys.stdin.fileno(), old_tty)
        send_zero_twist(stop_pub, times=5, sleep_s=0.05)
        rospy.logwarn("[KILL] Exiting policy loop. Robot commanded to stop.")

if __name__ == "__main__":
    main()
