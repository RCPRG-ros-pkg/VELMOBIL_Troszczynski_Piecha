from abc import ABC, abstractmethod
import optax
import flax.linen as nn

class ModelConfig(ABC):
    def __init__(self, drl_algorithm: str, total_timesteps: int):
        self.drl_algorithm = drl_algorithm
        self.total_timesteps = total_timesteps
        self.hiperparams: dict = {}
        self.net_arch: dict = {}

    @abstractmethod
    def __call__(self) -> tuple[int, dict, dict, nn.Module | None]:
        pass

    @classmethod
    def choose_model_config(cls, tc_name: str, total_timesteps: int) -> "ModelConfig":
        for subclass in cls.__subclasses__():
            if subclass.__name__ == tc_name:
                return subclass(total_timesteps)
        raise ValueError(f"Unknown model configuration name: {tc_name}")

# PPO algorithm configurations

class PPOConfig1(ModelConfig):
    def __init__(self, total_timesteps):
        super().__init__("PPO", total_timesteps)
        self.hiperparams["learning_rate"] = 3e-4
        self.hiperparams["batch_size"] = 64
        self.hiperparams["n_steps"] = 2048
        self.hiperparams["n_epochs"] = 10
        self.hiperparams["gamma"] = 0.99


        self.net_arch["pi"] = [128, 128]
        self.net_arch["vf"] = [128, 128]
    
    def __call__(self) -> tuple[int, dict, dict, nn.Module | None]:
        return (self.total_timesteps, self.hiperparams, dict(net_arch = self.net_arch, 
            activation_fn = nn.tanh, optimizer_class = optax.adam), None)


class PPOConfig2(ModelConfig):
    def __init__(self, total_timesteps):
        super().__init__("PPO", total_timesteps)
        self.hiperparams["learning_rate"] = 3e-4
        self.hiperparams["batch_size"] = 64
        self.hiperparams["n_steps"] = 2048
        self.hiperparams["n_epochs"] = 10
        self.hiperparams["gamma"] = 0.99

        self.net_arch["pi"] = [256, 256]
        self.net_arch["vf"] = [256, 256] 
    
    def __call__(self) -> tuple[int, dict, dict, nn.Module | None]:
        return (self.total_timesteps, self.hiperparams, dict(net_arch = self.net_arch, 
            activation_fn = nn.tanh, optimizer_class = optax.adam), None)


class PPOConfig3(ModelConfig):
    def __init__(self, total_timesteps):
        super().__init__("PPO", total_timesteps)
        self.hiperparams["learning_rate"] = 3e-4
        self.hiperparams["batch_size"] = 64
        self.hiperparams["n_steps"] = 2048
        self.hiperparams["n_epochs"] = 10
        self.hiperparams["gamma"] = 0.99

        self.net_arch["pi"] = [128, 128]
        self.net_arch["vf"] = [256, 256] # HEY I dont think Its gonna perform better. Most papers have same sizes of layers. They might only have a bit more layers.
    
    def __call__(self) -> tuple[int, dict, dict, nn.Module | None]:
        return (self.total_timesteps, self.hiperparams, dict(net_arch = self.net_arch, 
            activation_fn = nn.tanh, optimizer_class = optax.adam), None)


class PPOConfig4(ModelConfig):
    def __init__(self, total_timesteps):
        super().__init__("PPO", total_timesteps)
        self.hiperparams["learning_rate"] = 3e-4
        self.hiperparams["batch_size"] = 64
        self.hiperparams["n_steps"] = 2048
        self.hiperparams["n_epochs"] = 10
        self.hiperparams["gamma"] = 0.99

        self.net_arch["pi"] = [128, 128, 128]
        self.net_arch["vf"] = [128, 128, 128] 
    
    def __call__(self) -> tuple[int, dict, dict, nn.Module | None]:
        return (self.total_timesteps, self.hiperparams, dict(net_arch = self.net_arch, 
            activation_fn = nn.leaky_relu, optimizer_class = optax.adam), None)




# SAC algorithm configurations

class SACConfig1(ModelConfig):
    def __init__(self, total_timesteps):
        super().__init__("SAC", total_timesteps)
        self.hiperparams["learning_rate"] = 3e-4
        self.hiperparams["buffer_size"] = 300000
        self.hiperparams["batch_size"] = 256
        self.hiperparams["gamma"] = 0.99
        self.hiperparams["tau"] = 0.005
        self.hiperparams["train_freq"] = 1
        self.hiperparams["gradient_steps"] = 1
        self.hiperparams["ent_coef"] = "auto"

        self.net_arch["pi"] = [256, 256]
        self.net_arch["qf"] = [256, 256] 
    
    def __call__(self) -> tuple[int, dict, dict, nn.Module | None]:
        return (self.total_timesteps, self.hiperparams, dict(net_arch = self.net_arch, 
            activation_fn = nn.relu, optimizer_class = optax.adam), None)