from dataclasses import dataclass
from abc import ABC, abstractmethod
import numpy as np


@dataclass
class TrainingSignals:
    raw_lidar: np.ndarray
    goal_distance: float
    normalized_goal_distance: float
    normalized_bearing: float
    angular_velocity: float
    previous_normalized_goal_distance: float
    current_action_vector: np.ndarray
    previous_action_vector: np.ndarray


class RewardFunction(ABC):
    @abstractmethod
    def __call__(self, signals: TrainingSignals, collision_range: float, goal_tolerance: float) ->  tuple[float, bool, bool]:
        pass

    @classmethod
    def choose_rf(cls, rf_name: str) -> "RewardFunction":
        for subclass in cls.__subclasses__():
            if subclass.__name__ == rf_name:
                return subclass()
        raise ValueError(f"Uknown reward function name: {rf_name}")


class RewardFunction1(RewardFunction):
    def __call__(self, signals: TrainingSignals, collision_range: float, goal_tolerance: float) -> tuple[float, bool, bool]:
        reach_goal_weight = 10.0
        crash_weight = -10.0
        step_weight = 0.002
        progress_weight = 2.0

        collided = bool(np.any(signals.raw_lidar < collision_range))
        reached_goal = bool(signals.goal_distance < goal_tolerance)

        progress_reward = progress_weight * (signals.previous_normalized_goal_distance - signals.normalized_goal_distance)
        if reached_goal:
            reward = reach_goal_weight + progress_reward
        elif collided:
            reward = crash_weight
        else:
            reward = -step_weight + progress_reward
        return float(reward), collided, reached_goal


class RewardFunction2(RewardFunction):
    def __call__(self, signals: TrainingSignals, collision_range: float, goal_tolerance: float) -> tuple[float, bool, bool]:
        reach_goal_weight = 10.0
        crash_weight = -10.0
        step_weight = 0.002
        progress_weight = 2.0
        bearing_weight = 0.002

        collided = bool(np.any(signals.raw_lidar < collision_range))
        reached_goal = bool(signals.goal_distance < goal_tolerance)

        progress_reward = progress_weight * (signals.previous_normalized_goal_distance - signals.normalized_goal_distance)
        if reached_goal:
            reward = reach_goal_weight + progress_reward
        elif collided:
            reward = crash_weight
        else:
            bearing_reward = -bearing_weight * abs(signals.normalized_bearing)
            reward = -step_weight + progress_reward + bearing_reward
        return float(reward), collided, reached_goal


class RewardFunction3(RewardFunction):
    def __call__(self, signals: TrainingSignals, collision_range: float, goal_tolerance: float) -> tuple[float, bool, bool]:
        reach_goal_weight = 10.0
        crash_weight = -10.0
        step_weight = 0.002
        progress_weight = 2.0
        bearing_weight = 0.002              
        angular_velocity_weight = 0.000
        action_smoothness_weight = 0.02    

        collided = bool(np.any(signals.raw_lidar < collision_range))
        reached_goal = bool(signals.goal_distance < goal_tolerance)

        progress_reward = progress_weight * (signals.previous_normalized_goal_distance - signals.normalized_goal_distance)
        if reached_goal:
            reward = reach_goal_weight + progress_reward
        elif collided:
            reward = crash_weight
        else:
            bearing_reward = -bearing_weight * abs(signals.normalized_bearing)
            angular_velocity_reward = -angular_velocity_weight * signals.angular_velocity ** 2
            action_smoothness_reward = -action_smoothness_weight * np.sum((signals.current_action_vector - signals.previous_action_vector) ** 2)
            reward = -step_weight + progress_reward + bearing_reward + angular_velocity_reward + action_smoothness_reward
        return float(reward), collided, reached_goal
