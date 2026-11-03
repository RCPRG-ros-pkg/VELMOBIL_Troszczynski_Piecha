import os
import pandas as pd


SUCCESS_RATE_WINDOW = 100


#   Training
def save_training_episode(stage: str,
                            agent_training_drl_node,
                            training_output: list[dict],
                            training_logs_path: str = None) -> None:
    reached_goal = agent_training_drl_node.episode_outcome == "reached"
    success_history = [episode["reached"] for episode in training_output if episode["stage"] == stage and episode["outcome"] != "step_timeout"]
    if agent_training_drl_node.episode_outcome != "step_timeout":
        success_history.append(reached_goal)
    success_history = success_history[-SUCCESS_RATE_WINDOW:]
    episode_output = { "stage": stage,
        "start_pos_x": round(float(agent_training_drl_node.start_position[0]), 2), "start_pos_y": round(float(agent_training_drl_node.start_position[1]), 2),
        "goal_pos_x": round(float(agent_training_drl_node.goal_position[0]), 2), "goal_pos_y": round(float(agent_training_drl_node.goal_position[1]), 2),
        "reward": round(float(agent_training_drl_node.episode_reward), 2),
        "success_rate": round(100 * (sum(success_history) / len(success_history)), 2) if success_history else float("nan"),
        "time_to_goal": round(agent_training_drl_node.episode_time, 2) if reached_goal else float("nan"),
        "reached": reached_goal,
        "collided": agent_training_drl_node.episode_outcome == "collision",
        "outcome": agent_training_drl_node.episode_outcome
    }
    training_output.append(episode_output)
    if training_logs_path is None:
        return
    os.makedirs(os.path.dirname(training_logs_path), exist_ok = True)
    pd.DataFrame([episode_output]).to_csv(training_logs_path, mode = "a", index = False, header = not os.path.exists(training_logs_path))


def record_outcome_elapsed_time_while_training(agent_training_drl_node, outcome: str, elapsed: float) -> None:
    agent_training_drl_node.episode_outcome = outcome
    agent_training_drl_node.episode_time = elapsed


#   Inference
def save_trial_outputs(stage: str,
                        agent_inference_drl_node,
                        evaluation_output: list[dict],
                        evaluation_logs_path: str = None) -> None:
    reached_goal = agent_inference_drl_node.trial_outcome == "reached"
    trial_output = { "stage": stage,
        **agent_inference_drl_node.stage_parameters,
        "reached": reached_goal,
        "time_to_goal": round(agent_inference_drl_node.trial_time, 2) if reached_goal else float("nan"),
        "outcome": agent_inference_drl_node.trial_outcome,
        "start_pos_x": round(float(agent_inference_drl_node.start_position[0]), 2), "start_pos_y": round(float(agent_inference_drl_node.start_position[1]), 2),
        "goal_pos_x": round(float(agent_inference_drl_node.goal_position[0]), 2), "goal_pos_y": round(float(agent_inference_drl_node.goal_position[1]), 2),
        "goal_tolerance": float(agent_inference_drl_node.goal_tolerance)
    }
    evaluation_output.append(trial_output)
    if evaluation_logs_path is None:
        return
    os.makedirs(os.path.dirname(evaluation_logs_path), exist_ok = True)
    pd.DataFrame([trial_output]).to_csv(evaluation_logs_path, mode = "a", index = False, header = not os.path.exists(evaluation_logs_path))


def evaluation_summary(agent_inference_drl_node) -> None:
    results = pd.DataFrame(agent_inference_drl_node.evaluation_results)
    if results.empty:
        agent_inference_drl_node.get_logger().warn("[AgentInferenceDRL] No results")
        return
    valid = results[results["outcome"].isin(["reached", "collision", "timeout"])]
    if valid.empty:
        agent_inference_drl_node.get_logger().warn("[AgentInferenceDRL] No valid trials")
        return
    summary = valid.groupby("stage", sort = False).agg(
        trials=("reached", "size"),
        success_rate=("reached", "mean"),
        collisions=("outcome", lambda outcome: int((outcome == "collision").sum())),
        timeouts=("outcome", lambda outcome: int((outcome == "timeout").sum())),
        mean_time_to_goal=("time_to_goal", "mean"),
        std_time_to_goal=("time_to_goal", "std"),
    )
    agent_inference_drl_node.get_logger().info(f"[AgentInferenceDRL] Evaluation summary\n{summary.to_string(float_format='%.2f')}")


def record_outcome_elapsed_time_while_inference(agent_inference_drl_node, outcome: str, elapsed: float) -> None:
    agent_inference_drl_node.trial_outcome = outcome
    agent_inference_drl_node.trial_time = elapsed
