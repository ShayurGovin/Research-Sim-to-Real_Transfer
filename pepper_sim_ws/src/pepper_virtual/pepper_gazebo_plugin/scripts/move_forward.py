#!/usr/bin/env python3
import os
import sys
import time
import signal
import argparse
from datetime import datetime

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import (
    DummyVecEnv,
    SubprocVecEnv,
    VecNormalize,
    VecMonitor,
)
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.utils import set_random_seed

sys.path.append('/home/shaytrix2/workspace/pepper_sim_ws/src')
from gym_pepper_env.gym_env.pepper_env import make_improved_env


class TimeCheckpointCallback(BaseCallback):
    def __init__(self, save_dir: str, name_prefix: str = "ppo_pepper",
                 save_interval_sec: int = 300, verbose: int = 1):
        super().__init__(verbose)
        os.makedirs(save_dir, exist_ok=True)
        self.save_dir = save_dir
        self.name_prefix = name_prefix
        self.interval = max(60, int(save_interval_sec))
        self._last = time.time()

    def _init_callback(self) -> None:
        self.model.save(os.path.join(self.save_dir, "latest"))
        self._last = time.time()

    def _on_step(self) -> bool:
        now = time.time()
        if now - self._last >= self.interval:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.save_dir, f"{self.name_prefix}_time_{ts}")
            if self.verbose:
                print(f"[TimeCheckpoint] -> {path}.zip")
            self.model.save(path)
            self.model.save(os.path.join(self.save_dir, "latest"))
            self._last = now
        return True



class SaveOnInterrupt:
    def __init__(self, model, save_dir: str, tag: str = "interrupt"):
        self.model = model
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        self._old = {}

        def handler(signum, frame):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.save_dir, f"{tag}_{ts}")
            print(f"\n[Signal {signum}] Saving -> {path}.zip and latest.zip")
            self.model.save(path)
            self.model.save(os.path.join(self.save_dir, "latest"))
            sys.exit(0)

        self._handler = handler

    def __enter__(self):
        for s in (signal.SIGINT, signal.SIGTERM):
            self._old[s] = signal.getsignal(s)
            signal.signal(s, self._handler)

    def __exit__(self, exc_type, exc, tb):
        for s, h in self._old.items():
            signal.signal(s, h)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.save_dir, f"final_{ts}")
        print(f"\n[Exit] Saving -> {path}.zip and latest.zip")
        self.model.save(path)
        self.model.save(os.path.join(self.save_dir, "latest"))


def build_vec_env(n_envs: int, seed: int, env_kwargs: dict, subproc: bool):
    set_random_seed(seed)
    thunks = [make_improved_env(**env_kwargs) for _ in range(n_envs)]
    Vec = SubprocVecEnv if (subproc and n_envs > 1) else DummyVecEnv
    return Vec(thunks)


def main():
    ap = argparse.ArgumentParser("Train Pepper to pursue red object without collision (PPO) + random spawn")


    ap.add_argument("--total_timesteps", type=int, default=2_000_000)
    ap.add_argument("--n_envs", type=int, default=1)
    ap.add_argument("--subproc", action="store_true")
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--normalize", action="store_true")

    ap.add_argument("--logdir", type=str, default="./runs/ppo_pepper")
    ap.add_argument("--ckpt_dir", type=str, default="./ckpts")
    ap.add_argument("--best_dir", type=str, default="./best")

    ap.add_argument("--save_freq_steps", type=int, default=100_000)
    ap.add_argument("--save_interval_min", type=int, default=5)

    ap.add_argument("--eval_every_steps", type=int, default=25_000)
    ap.add_argument("--eval_episodes", type=int, default=5)

    ap.add_argument("--policy", type=str, default="MlpPolicy", choices=["MlpPolicy", "CnnPolicy"])
    ap.add_argument("--gamma", type=float, default=0.995)
    ap.add_argument("--gae_lambda", type=float, default=0.95)
    ap.add_argument("--learning_rate", type=float, default=3e-4)
    ap.add_argument("--ent_coef", type=float, default=0.0)
    ap.add_argument("--vf_coef", type=float, default=0.5)
    ap.add_argument("--clip_range", type=float, default=0.2)
    ap.add_argument("--n_steps", type=int, default=4096)
    ap.add_argument("--batch_size", type=int, default=2048)
    ap.add_argument("--n_epochs", type=int, default=10)

    ap.add_argument("--image_topic", type=str, default="/pepper/camera/front/image_raw")
    ap.add_argument("--caminfo_topic", type=str, default="/pepper/camera/front/camera_info")
    ap.add_argument("--scan_topic", type=str, default="/pepper/scan_front")
    ap.add_argument("--cmd_topic", type=str, default="/pepper/cmd_vel")
    ap.add_argument("--max_episode_steps", type=int, default=600)

    ap.add_argument("--model_name", type=str, default="pepper_MP", help="Gazebo model name")
    ap.add_argument("--disable_random_spawn", action="store_true",
                    help="Do not teleport at reset() in the training env")
    ap.add_argument("--spawn_positions", type=str, default=None,
                    help='JSON list of {"x":..., "y":...}. '
                         'Default = [(-2,-1.7),(-1.3,-4.597),(-3,0.4),(0,-1)]')

    args = ap.parse_args()

    tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    logdir   = os.path.join(args.logdir, tag)
    ckpt_dir = os.path.join(args.ckpt_dir, tag)
    best_dir = os.path.join(args.best_dir, tag)
    os.makedirs(logdir, exist_ok=True)
    os.makedirs(ckpt_dir, exist_ok=True)
    os.makedirs(best_dir, exist_ok=True)

    spawn_positions = None
    if args.spawn_positions:
        import json
        try:
            raw = json.loads(args.spawn_positions)
            spawn_positions = []
            for p in raw:
                spawn_positions.append({"x": float(p["x"]), "y": float(p["y"])})
            print(f"Using custom spawn positions: {spawn_positions}")
        except Exception as e:
            print(f"Warning: could not parse --spawn_positions JSON ({e}); using defaults")

    env_kwargs = dict(
        image_topic=args.image_topic,
        caminfo_topic=args.caminfo_topic,
        scan_topic=args.scan_topic,
        cmd_topic=args.cmd_topic,
        max_episode_steps=args.max_episode_steps,
        model_name=args.model_name,
        enable_random_spawn=not args.disable_random_spawn,
        spawn_positions=spawn_positions,
        debug=(args.n_envs == 1),
    )

    print("Environment configuration:")
    print(f"  Random spawning: {'Enabled' if not args.disable_random_spawn else 'Disabled'}")
    print(f"  Model name: {args.model_name}")
    print(f"  Max episode steps: {args.max_episode_steps}")
    print(f"  Number of environments: {args.n_envs}")

    train_env = build_vec_env(args.n_envs, args.seed, env_kwargs, subproc=args.subproc)

    eval_env_kwargs = env_kwargs.copy()
    eval_env_kwargs["debug"] = False
    eval_env_kwargs["enable_random_spawn"] = False
    eval_env  = build_vec_env(1, args.seed + 1000, eval_env_kwargs, subproc=False)

    train_env = VecMonitor(train_env)
    eval_env  = VecMonitor(eval_env)

    if args.normalize:
        train_env = VecNormalize(train_env, norm_obs=True, norm_reward=True, clip_obs=10.0)
        eval_env  = VecNormalize(eval_env, training=False, norm_obs=True, norm_reward=False, clip_obs=10.0)

    model = PPO(
        policy=args.policy,
        env=train_env,
        tensorboard_log=logdir,
        seed=args.seed,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        learning_rate=args.learning_rate,
        ent_coef=args.ent_coef,
        vf_coef=args.vf_coef,
        clip_range=args.clip_range,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        verbose=1,
    )

    step_ckpt = CheckpointCallback(
        save_freq=max(10_000, args.save_freq_steps),
        save_path=ckpt_dir,
        name_prefix="ppo_pepper_steps",
        save_vecnormalize=True,
        save_replay_buffer=False,
    )

    time_ckpt = TimeCheckpointCallback(
        save_dir=ckpt_dir,
        name_prefix="ppo_pepper",
        save_interval_sec=int(args.save_interval_min * 60),
        verbose=1,
    )

    if isinstance(train_env, VecNormalize) and isinstance(eval_env, VecNormalize):
        eval_env.obs_rms = train_env.obs_rms

    eval_cb = EvalCallback(
        eval_env=eval_env,
        best_model_save_path=best_dir,
        log_path=best_dir,
        n_eval_episodes=max(3, args.eval_episodes),
        eval_freq=max(10_000, args.eval_every_steps),
        deterministic=True,
        render=False,
    )

    with SaveOnInterrupt(model, save_dir=ckpt_dir):
        model.learn(total_timesteps=args.total_timesteps, callback=[step_ckpt, time_ckpt, eval_cb])

    if isinstance(train_env, VecNormalize):
        train_env.save(os.path.join(ckpt_dir, "vecnormalize.pkl"))

    model.save(os.path.join(ckpt_dir, "final"))


if __name__ == "__main__":
    main()
