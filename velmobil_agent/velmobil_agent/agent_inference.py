#!/usr/bin/env python3
import rclpy
from rclpy.executors import MultiThreadedExecutor
from .AgentInferenceDRL import AgentInferenceDRL
from .Model import Model


MODEL_LOAD_PATH = "src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/PPO_5_trained_policy.zip"


def main():
    rclpy.init()
    model = Model(model_path = MODEL_LOAD_PATH)
    agent_inference_drl_node = AgentInferenceDRL(model = model)
    executor = MultiThreadedExecutor()
    executor.add_node(agent_inference_drl_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        agent_inference_drl_node.stop_robot()
        executor.shutdown()
        agent_inference_drl_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()