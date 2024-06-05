import json

from matplotlib import pyplot as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-report', help='path to the report', required=True)
args = parser.parse_args()
report_path = args.report

with open(f'{report_path}', 'r') as file:
    experiment_results = json.load(file)
    
scenes = experiment_results.keys()

# Initialize global dictionaries to hold the average values
avg_time = {scene: {lm: {} for lm in experiment_results[scene].keys()} for scene in scenes}
success_rate = {scene: {lm: {} for lm in experiment_results[scene].keys()} for scene in scenes}
avg_distance = {scene: {lm: {} for lm in experiment_results[scene].keys()} for scene in scenes}

# Loads experiments results to the
def loadResultsToDicts():
    for scene in scenes:
        for lm in experiment_results[scene].keys():
            for method, metrics in experiment_results[scene][lm].items():
                avg_time[scene][lm][method] = metrics['avg time']
                success_rate[scene][lm][method] = metrics['success rate']
                avg_distance[scene][lm][method] = metrics['avg distance']

# Function to plot bar charts for a given metric
def plotMetric(metric_data, metric_name):
    fig, axs = plt.subplots(len(scenes), 1, figsize=(7, len(scenes)*2))
    if len(scenes) == 1:
        axs = [axs]  # Ensure axs is always iterable
    
    for idx, (scene, lm_data) in enumerate(metric_data.items()):
        ax = axs[idx]
        methods = list(next(iter(lm_data.values())).keys())
        x = np.arange(len(experiment_results[scene].keys()))
        width = 0.15

        for i, method in enumerate(methods):
            values = [lm_data[lm][method] for lm in experiment_results[scene].keys()]
            ax.bar(x + i * width, values, width, label=method)

        title = ax.set_title(f'{metric_name}-{scene}',fontweight='bold')
        title.set_position([0.32, 1.05])
        title.set_fontsize(10)
        ax.set_xticks(x + width)
        ax.set_xticklabels(list(experiment_results[scene].keys()),fontsize=8)
        if metric_name=="Average Distance":
            ax.set_ylim(0, 300)
            tick_positions = range(0, 201, 50 )
            ax.set_yticks(tick_positions)
        ax.legend()

    fig.tight_layout()
    fig.subplots_adjust(top=0.9, hspace=0.4) 
    plt.show()
    

if __name__ == "__main__":
    loadResultsToDicts()
    plotMetric(success_rate, 'Success Rate')
    plotMetric(avg_distance, 'Average Distance')