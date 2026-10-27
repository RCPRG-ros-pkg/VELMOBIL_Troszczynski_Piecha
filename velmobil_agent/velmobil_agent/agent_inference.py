#!/usr/bin/env python3
import os
import rclpy
import threading
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor
from .AgentInferenceDRL import AgentInferenceDRL
from .Model import Model


MODEL_LOAD_PATH = "src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/trained_policies/PPO_5_trained_policy.zip"
EVALUATION_SAVE_DIR = "src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/evaluation_logs"


def main():
    rclpy.init()
    evaluation_save_path = os.path.join(EVALUATION_SAVE_DIR, f"PPO_{datetime.now().strftime('%d%m%Y_%H%M')}.csv")

    model = Model(model_path = MODEL_LOAD_PATH)
    agent_inference_drl_node = AgentInferenceDRL(model = model, evaluation_save_path = evaluation_save_path)
    executor = MultiThreadedExecutor()
    executor.add_node(agent_inference_drl_node)

    spin_thread = threading.Thread(target = executor.spin, daemon = True)
    spin_thread.start()
    try:
        while rclpy.ok() and not agent_inference_drl_node.done_evaluation.wait(timeout=1.0):
            pass
        agent_inference_drl_node.get_logger().info(f"Evaluation results saved: {evaluation_save_path}")
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
