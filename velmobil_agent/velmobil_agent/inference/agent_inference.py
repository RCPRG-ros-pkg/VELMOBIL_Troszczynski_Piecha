#!/usr/bin/env python3
import os
import rclpy
import threading
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from velmobil_agent.inference.AgentInferenceDRL import AgentInferenceDRL
from velmobil_agent.common.Model import Model


MODEL_LOAD_PATH = "src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/PPO_trained_policy_10112026_2221.zip"
EVALUATION_SAVE_PATH = f"src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/evaluation_logs/PPO_{datetime.now().strftime('%d%m%Y_%H%M')}.csv"


def main():
    rclpy.init()
    model = Model(model_path = MODEL_LOAD_PATH)
    agent_inference_drl_node = AgentInferenceDRL(model = model, evaluation_save_path = EVALUATION_SAVE_PATH)
    executor = MultiThreadedExecutor()
    executor.add_node(agent_inference_drl_node)

    spin_thread = threading.Thread(target = executor.spin, daemon = True)
    spin_thread.start()
    try:
        while rclpy.ok() and not agent_inference_drl_node.done_evaluation.wait(timeout=1.0):
            pass
        agent_inference_drl_node.get_logger().info(f"Evaluation results saved: {EVALUATION_SAVE_PATH}")
    except KeyboardInterrupt:
        pass
    finally:
        agent_inference_drl_node.stop_robot()
        executor.shutdown()
        agent_inference_drl_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
