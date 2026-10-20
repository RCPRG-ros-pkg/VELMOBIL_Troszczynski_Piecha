#!/usr/bin/env python3
import rclpy
import numpy as np
import threading
from rclpy.executors import MultiThreadedExecutor
from .AgentTrainingDRL import AgentTrainingDRL
from .Model import Model
import matplotlib.pyplot as plt

TOTAL_TIMESTEPS = 300000
MODEL_SAVE_PATH = "src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/PPO_fourth_trained_policy"

def goal_sampler() -> np.ndarray:
    # simple randomness. Needs improvement...
    return np.concatenate([np.random.uniform(5, 8, 2) * np.sign(np.random.uniform(-1.0, 1.0, 2)), [0]])

def main():
    rclpy.init()
    agent_training_drl_node = AgentTrainingDRL()
    executor = MultiThreadedExecutor()
    executor.add_node(agent_training_drl_node)

    # executor musi spinować równolegle do model.learn(), bo env.step() wewnątrz
    # niego blokuje się na danych z lidar_sub/odom_sub - te callbacki inaczej nigdy by się nie wykonały
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    model = Model(
        device="cuda",
        env=agent_training_drl_node, 
        model_path=None, 
        learning_rate=3e-4,
        batch_size=64,
        n_steps=2048,
        n_epochs=10,
        policy_kwargs=dict(net_arch=[128,128]),
        gamma=0.99,
        verbose=1,
        seed=0
    )

    try:
        model.train(total_timesteps=TOTAL_TIMESTEPS)
        model.save(MODEL_SAVE_PATH)
        rewards = np.array(agent_training_drl_node.episode_rewards)
        window = 20
        plt.figure()
        plt.plot(rewards, alpha=0.3, label='Episode reward')
        if len(rewards) >= window:
            moving_avg = np.convolve(rewards, np.ones(window) / window, mode='valid')
            plt.plot(np.arange(window - 1, len(rewards)), moving_avg, label=f'Moving average')
        plt.title('Episode rewards')
        plt.xlabel('Episode')
        plt.ylabel('Episode reward')
        plt.legend()
        plt.grid()
        plt.tight_layout()
        plt.savefig('/root/ws/src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/plots/PPO_fourth_episode_rewards.png')
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        agent_training_drl_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)



if __name__ == "__main__":
    main()
