#!/usr/bin/env python3
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from .AgentTrainingDRL import AgentTrainingDRL
from velmobil_agent.common.Model import Model
from datetime import datetime
from velmobil_agent.common.model_configuration import ModelConfig


TIME = datetime.now().strftime('%d%m%Y_%H%M')
TRAINING_SAVE_PATH = f"src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/"


def main():
    rclpy.init()
    agent_training_drl_node = AgentTrainingDRL(training_save_path=TRAINING_SAVE_PATH)
    executor = MultiThreadedExecutor()
    executor.add_node(agent_training_drl_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()



    model = Model(env=agent_training_drl_node, 
                  model_path=None, 
                  algorithm_name=agent_training_drl_node.get_parameter('algorithm_name').value, 
                  algorithm_configuration=agent_training_drl_node.get_parameter('algorithm_configuration').value,
                  total_timesteps=agent_training_drl_node.get_parameter('total_timesteps').value,
                  verbose=1,
                  seed=agent_training_drl_node.get_parameter('seed').value)
    try:
        model.train()
        model_save_filepath = f"src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/{agent_training_drl_node.get_parameter('algorithm_name').value}_trained_policy_{TIME}"
        model.save(model_save_filepath)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        agent_training_drl_node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)



if __name__ == "__main__":
    main()
