#!/usr/bin/env python3
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from .AgentTrainingDRL import AgentTrainingDRL
from velmobil_agent.common.Model import Model, ALGORITHM, ALGORITHM_NAME
from datetime import datetime
from velmobil_agent.common.model_configuration import ModelConfig


TIME = datetime.now().strftime('%d%m%Y_%H%M')

MODEL_SAVE_PATH = f"src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/{ALGORITHM_NAME}_trained_policy_{TIME}"
TRAINING_SAVE_PATH = f"src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/{ALGORITHM_NAME}_{TIME}.csv"


def main():
    rclpy.init()
    agent_training_drl_node = AgentTrainingDRL(training_save_path = TRAINING_SAVE_PATH)
    executor = MultiThreadedExecutor()
    executor.add_node(agent_training_drl_node)

    # executor musi spinować równolegle do model.learn(), bo env.step() wewnątrz
    # niego blokuje się na danych z lidar_sub/odom_sub - te callbacki inaczej nigdy by się nie wykonały
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    model = Model(env=agent_training_drl_node, 
                  model_path=None, 
                  algorithm=ALGORITHM, 
                  verbose=1,
                  seed=agent_training_drl_node.get_parameter('seed').value)
    try:
        model.train()
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
