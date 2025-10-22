#!/usr/bin/env python3
import subprocess, signal, time, os, datetime

TOPICS = [
    "/pepper/cmd_vel",
    "/pepper/odom",
    "/pepper/scan_front",
    "/pepper/camera/front/image_raw",
    "/pepper/camera/front/camera_info"
]

def start_rosbag(out_dir=".", prefix="pepper_eval"):
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    bagfile = os.path.join(out_dir, f"{prefix}_{stamp}.bag")
    cmd = ["rosbag", "record", "-O", bagfile] + TOPICS
    print(f"[rosbag] Recording -> {bagfile}")
    p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return p, bagfile

def stop_rosbag(p):
    if p.poll() is None:
        print("[rosbag] Stopping...")
        p.send_signal(signal.SIGINT)
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.terminate()

if __name__ == "__main__":
    proc, bag = start_rosbag(out_dir="bags")
    try:
        print("[rosbag] Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_rosbag(proc)
        print(f"[rosbag] Saved {bag}")
