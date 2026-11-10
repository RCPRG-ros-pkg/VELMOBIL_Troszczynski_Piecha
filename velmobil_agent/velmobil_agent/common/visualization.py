import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


# Training plots

def plot_rewards(dfs: list[tuple[str, pd.DataFrame]], 
                save_path: str,
                smoothing_window: int):
    fig, ax = plt.subplots()
    for label, df in dfs:
        series = df['reward'].reset_index(drop = True)
        if smoothing_window > 1:
            series = series.rolling(smoothing_window, min_periods = 1).mean()
        ax.plot(series, label = label)
    ax.set_title('Reward per episode')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Reward')
    ax.grid()
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close()
    


def plot_success_rate():
    pass


def plot_time_to_goal():
    pass



def visualization():
    dfs = [('PPO', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/PPO_03112026_0040.csv')), ('SAC', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/SAC_09112026_2123.csv'))]
    plot_rewards(dfs = dfs, save_path = 'src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/plots/RewardsPerEpisode.png', smoothing_window = 1)


if __name__ == "__main__":
    visualization()