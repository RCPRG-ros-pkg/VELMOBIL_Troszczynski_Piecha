import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


#   Training plots
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


# Evaluation plots
def stage_positions(dfs: list[tuple[str, pd.DataFrame]]):
    stages = sorted(set().union(*(df['stage'].unique() for _, df in dfs)))
    width = 0.8 / len(dfs)
    x = np.arange(len(stages))
    offsets = [(i - (len(dfs)  -1) / 2) * width for i in range(len(dfs))]
    return stages, x, width, offsets


def plot_evaluation_outcome(dfs: list[tuple[str, pd.DataFrame]], 
                            save_path: str):
    outcomes = {'reached': '#2ca02c', 'collision': '#d62728', 'timeout': '#7f7f7f'}
    stages, x, width, offsets = stage_positions(dfs)
    fig, ax = plt.subplots()
    tick_positions, tick_labels = [], []
    for (label, df), offset in zip(dfs, offsets):
        positions = x + offset
        bottom = np.zeros(len(stages))
        for outcome, color in outcomes.items():
            share = np.array([100 * (df.loc[df['stage'] == stage, 'outcome'] == outcome).sum() / max((df['stage'] == stage).sum(), 1) for stage in stages])
            ax.bar(positions, share, width, bottom = bottom, color = color, label = outcome if label == dfs[0][0] else None)
            bottom += share
        tick_positions.extend(positions)
        tick_labels.extend(stage if len(dfs) == 1 else f'{stage}\n{label}' for stage in stages)
    ax.set_xticks(tick_positions, tick_labels)
    ax.set_title('Trial outcomes per stage')
    ax.set_xlabel('Stage')
    ax.set_ylabel('Share of trials [%]')
    ax.set_ylim(0, 100)
    ax.grid(axis = 'y')
    ax.legend()
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close()


def plot_evaluation_time_to_goal(dfs: list[tuple[str, pd.DataFrame]],
                                 save_path: str):
    stages, x, width, offsets = stage_positions(dfs)
    fig, ax = plt.subplots()
    handles = []
    for i, ((label, df), offset) in enumerate(zip(dfs, offsets)):
        data = [df.loc[df['stage'] == stage, 'time_to_goal'].dropna() for stage in stages]
        box = ax.boxplot(data, positions = x + offset, widths = width * 0.9, patch_artist = True, manage_ticks = False)
        for patch in box['boxes']:
            patch.set_facecolor(f'C{i}')
        for median in box['medians']:
            median.set_color('black')
        handles.append(box['boxes'][0])
    ax.set_xticks(x, stages)
    ax.set_title('Time to goal per stage - reached trials')
    ax.set_xlabel('Stage')
    ax.set_ylabel('Time to goal [s]')
    ax.grid(axis = 'y')
    ax.legend(handles, [label for label, _ in dfs])
    fig.tight_layout()
    fig.savefig(save_path)
    plt.close()


def visualization():
    plot_folder = 'src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/plots' 
    dfs = [('PPO', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/PPO_10112026_2221.csv')), ('SAC', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/training_logs/SAC_09112026_2123.csv'))]
    eval_dfs = [('PPO', pd.read_csv('src/VELMOBIL_Troszczynski_Piecha/velmobil_agent/logs/evaluation_logs/PPO_11112026_0020.csv'))]
    plot_rewards(dfs = dfs, save_path = f'{plot_folder}/RewardsPerEpisode_2.png', smoothing_window = 1)
    plot_success_rate(dfs = dfs, save_path = f'{plot_folder}/SuccessRatePerEpisode_2.png')
    plot_time_to_goal(dfs = dfs, save_path = f'{plot_folder}/TimeToGoalPerEpisode_2.png')
    plot_evaluation_outcome(dfs = eval_dfs, save_path = f'{plot_folder}/OutcomePerStage.png')
    plot_evaluation_time_to_goal(dfs = eval_dfs, save_path = f'{plot_folder}/TimeToGoalPerStage.png')


if __name__ == "__main__":
    visualization()