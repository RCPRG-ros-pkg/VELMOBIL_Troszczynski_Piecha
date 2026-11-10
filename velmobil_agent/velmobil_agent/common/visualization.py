import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def plot_rewards(dfs: list[tuple[str, pd.DataFrame]], 
                save_path: str,
                smoothing_window: int):
    series_list = []
    for label, df in dfs:
        series = df['reward'].reset_index(drop = True)
        if smoothing_window > 1:
            series = series.rolling(smoothing_window, min_periods = 1).mean()
        series_list.append((label, series))
    if series_list:
        min_len = min(len(series) for _, series in series_list)
        series_list = [(label, series.iloc[:min_len]) for label, series in series_list]
    fig, ax = plt.subplots()
    for label, series in series_list:
        ax.plot(series, label = label)
    ax.set_title('Reward per episode')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Reward')
    ax.grid()
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close()
    

def plot_success_rate(dfs: list[tuple[str, pd.DataFrame]], 
                    save_path: str):
    series_list = [(label, df['success_rate'].reset_index(drop = True)) for label, df in dfs]
    if series_list:
        min_len = min(len(series) for _, series in series_list)
        series_list = [(label, series.iloc[:min_len]) for label, series in series_list]
    fig, ax = plt.subplots()
    for label, series in series_list:
        ax.plot(series, label = label)
    ax.set_title('Success rate per episode')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Success rate [%]')
    ax.set_ylim(0, 100)
    ax.grid()
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close()


def plot_time_to_goal(dfs: list[tuple[str, pd.DataFrame]],
                      save_path: str,
                      smoothing_window: int = 50):
    series_list = []
    for label, df in dfs:
        series = df['time_to_goal'].reset_index(drop = True)
        if smoothing_window > 1:
            series = series.rolling(smoothing_window, min_periods = 1).mean()
        series_list.append((label, series))
    if series_list:
        min_len = min(len(series) for _, series in series_list)
        series_list = [(label, series.iloc[:min_len]) for label, series in series_list]
    fig, ax = plt.subplots()
    for label, series in series_list:
        ax.plot(series, label = label)
    ax.set_title('Time to goal per episode')
    ax.set_xlabel('Episode')
    ax.set_ylabel('Time to goal [s]')
    ax.grid()
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close()


def visualization():
    plot_folder = 'src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/plots' 
    dfs = [('PPO', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/PPO_10112026_2221.csv')), ('SAC', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/SAC_09112026_2123.csv'))]
    plot_rewards(dfs = dfs, save_path = f'{plot_folder}/RewardsPerEpisode_2.png', smoothing_window = 1)
    plot_success_rate(dfs = dfs, save_path = f'{plot_folder}/SuccessRatePerEpisode_2.png')
    plot_time_to_goal(dfs = dfs, save_path = f'{plot_folder}/TimeToGoalPerEpisode_2.png')


if __name__ == "__main__":
    visualization()