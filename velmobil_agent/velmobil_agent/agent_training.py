#!/usr/bin/env python3
import rclpy
import numpy as np
import threading
from rclpy.executors import MultiThreadedExecutor
from .AgentTrainingDRL import AgentTrainingDRL
from .Model import Model

TOTAL_TIMESTEPS = 100_000
MODEL_SAVE_PATH = "src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/velmobil_trained_policy"

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
        learning_starts=5_000, 
        train_freq=(1, "episode"), 
        gradient_steps=-1,
        batch_size=256,
        learning_rate=3e-4,
        verbose=1
    )

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
