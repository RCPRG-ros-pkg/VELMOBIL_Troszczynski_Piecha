import os
import pandas as pd


def save_trial_outputs(stage: str,
                        agent_inference_drl_node,
                        evaluation_output: list[dict],
                        evaluation_logs_path: str = None) -> None:
    reached_goal = agent_inference_drl_node.trial_outcome == "reached"
    trial_output = { "stage": stage,
        **agent_inference_drl_node.stage_parameters,
        "reached": reached_goal,
        "time_to_goal": agent_inference_drl_node.trial_time if reached_goal else float("nan"),
        "outcome": agent_inference_drl_node.trial_outcome,
        "start_pos_x": float(agent_inference_drl_node.start_position[0]), "start_pos_y": float(agent_inference_drl_node.start_position[1]),
        "goal_pos_x": float(agent_inference_drl_node.goal_position[0]), "goal_pos_y": float(agent_inference_drl_node.goal_position[1]),
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


def record_outcome_elapsed_time(agent_inference_drl_node, outcome: str, elapsed: float) -> None:
    agent_inference_drl_node.trial_outcome = outcome
    agent_inference_drl_node.trial_time = elapsed
