# Copyright (c) 2025, The Berkeley Humanoid Lite Project Developers.

import time
import json
import argparse

import numpy as np
from cc.udp import UDP
from omegaconf import DictConfig, ListConfig, OmegaConf


# Parse config
def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=str, default="policy.yaml")
    args = parser.parse_args()

    print("Loading config file from ", args.config)

    with open(args.config, "r") as f:
        cfg = OmegaConf.load(f)

    return cfg

np.set_printoptions(precision=2)


class PolicyRunner:
    @staticmethod
    def quat_rotate_inverse(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        q_w = q[0]
        q_vec = q[1:4]
        a = v * (2.0 * q_w ** 2 - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        c = q_vec * (np.dot(q_vec, v)) * 2.0
        return a - b + c

    def __init__(self, cfg: DictConfig | ListConfig):
        self.cfg = cfg

    def run(self) -> None:
        
        model_checkpoint_path = self.cfg.policy_checkpoint_path

        runner = None

        if ".pt" in model_checkpoint_path:
            import torch
            torch.set_printoptions(precision=2)
            runner = "torch"
            print("Using Torch runner")
            
        elif ".onnx" in model_checkpoint_path:
            import onnxruntime as ort
            runner = "onnx"
            print("Using ONNX runner")

        if not runner:
            raise ValueError("Unrecognized policy format")


        command_velocity = np.array(self.cfg.command_velocity, dtype=np.float32)

        if self.cfg.num_actions == self.cfg.num_joints:
            default_joint_positions = np.array(self.cfg.default_joint_positions, dtype=np.float32)
        else:
            default_joint_positions = np.array(self.cfg.default_joint_positions[10:], dtype=np.float32)

        gravity_vector = np.array([0., 0., -1.], dtype=np.float32)

        policy_observations = np.zeros((1, self.cfg.num_observations * (self.cfg.history_length + 1)), dtype=np.float32)
        policy_actions = np.zeros((1, self.cfg.num_actions), dtype=np.float32)
        prev_actions = np.zeros((self.cfg.num_actions,), dtype=np.float32)


        if runner == "torch":
            policy_pt: torch.nn.Module = torch.load(self.cfg.policy_checkpoint_path)

        if runner == "onnx":
            policy_onnx: ort.InferenceSession = ort.InferenceSession(self.cfg.policy_checkpoint_path)

            try:
                policy_actions[:] = policy_onnx.run(None, {"obs": np.zeros((1, self.cfg.num_observations), dtype=np.float32)})[0]
                key = "obs"
            except Exception as e:
                print(e)
                key = "onnx::Gemm_0"

        udp = UDP((self.cfg.ip_host_addr, self.cfg.ip_policy_obs_port), (self.cfg.ip_robot_addr, self.cfg.ip_policy_acs_port))


        data_log = []

        counter = 0
        try:
            while True:
                udp_observations = udp.recv_numpy(dtype=np.float32)

                udp_base_quat = np.array(udp_observations[0:4], dtype=np.float32)
                udp_base_ang_vel = np.array(udp_observations[4:7], dtype=np.float32)
                udp_joint_pos = np.array(udp_observations[7:7+self.cfg.num_actions], dtype=np.float32) - default_joint_positions
                udp_joint_vel = np.array(udp_observations[7+self.cfg.num_actions:7+self.cfg.num_actions*2], dtype=np.float32)
                udp_mode = udp_observations[7+self.cfg.num_actions*2]
                command_velocity = np.array(udp_observations[7+self.cfg.num_actions*2+1:7+self.cfg.num_actions*2+4], dtype=np.float32)

                # if not in RL running mode, skip
                if udp_mode != 3:
                    print(f"Mode is not 3, skipping: {udp_mode}")
                    continue

                # Update observation buffer
                base_ang_vel = udp_base_ang_vel
                projected_gravity = self.quat_rotate_inverse(udp_base_quat, gravity_vector)
                joint_pos = udp_joint_pos
                joint_vel = udp_joint_vel

                policy_observations[:] = np.concatenate([
                    policy_observations[0, self.cfg.num_observations:],
                    command_velocity,
                    base_ang_vel,
                    projected_gravity,
                    joint_pos,
                    joint_vel,
                    prev_actions
                    ], axis=0)
                
                
                if runner == "torch":
                    policy_observations_tensor = torch.tensor(policy_observations)
                    
                    policy_actions[:] = policy_pt.forward(policy_observations_tensor).detach().numpy()

                if runner == "onnx":
                    policy_actions[:] = policy_onnx.run(None, {key: policy_observations})[0]

                # this creates a new tensor
                policy_actions_clipped = np.clip(policy_actions[0], self.cfg.action_limit_lower, self.cfg.action_limit_upper)

                # update observation buffer
                prev_actions[:] = policy_actions_clipped

                policy_actions_scaled = policy_actions_clipped * self.cfg.action_scale + default_joint_positions

                udp_actions = policy_actions_scaled

                udp.send_numpy(udp_actions)

                # log data
                data_log.append(np.concatenate([[time.time()], policy_observations.flatten()]).tolist())

                
                end_time = time.time()


                counter += 1


                # print(command_velocity)
                # print(f"Time taken: {(end_time - start_time)*1000.0:.2f} ms, {1/(end_time - start_time):.2f} Hz, actions: ")

            
        except KeyboardInterrupt:
            print("Keyboard interrupt detected. Exiting...")

        with open("data_log.json", "w") as f:
            json.dump(data_log, f)

        print("Written experiment data to ./data_log.json")
            

if __name__ == "__main__":
    cfg = parse_arguments()

    if not cfg:
        raise ValueError(f"Failed to load config.")

    runner = PolicyRunner(cfg)

    runner.run()


