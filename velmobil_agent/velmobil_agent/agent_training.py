#!/usr/bin/env python3
import rclpy
import numpy as np
import threading
from rclpy.executors import MultiThreadedExecutor
from .AgentTrainingDRL import AgentTrainingDRL
from .Model import Model

TOTAL_TIMESTEPS = 100_000
MODEL_SAVE_PATH = "velmobil_trained_model"

def goal_sampler() -> np.ndarray:
    # simple randomness. Needs improvement...
    return np.concatenate([np.random.uniform(5, 8, 2) * np.sign(np.random.uniform(-1.0, 1.0, 2)), [0]])

def main():
    rclpy.init()
    agent_training_drl_node = AgentTrainingDRL(goal_sampler=goal_sampler)
    executor = MultiThreadedExecutor()
    executor.add_node(agent_training_drl_node)

    # executor musi spinować równolegle do model.learn(), bo env.step() wewnątrz
    # niego blokuje się na danych z lidar_sub/odom_sub - te callbacki inaczej nigdy by się nie wykonały
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    model = Model(env=agent_training_drl_node, model_path=None)

    try:
        model.train(total_timesteps=TOTAL_TIMESTEPS)
        model.save(MODEL_SAVE_PATH)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        agent_training_drl_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)



if __name__ == "__main__":
    main()
