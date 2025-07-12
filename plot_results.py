import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_mean_flow_rate_per_multiplier(file_path="simulation_results.csv"):
    if not os.path.exists(file_path):
        print(f"Error: File not found at {file_path}")
        return

    df = pd.read_csv(file_path)

    # Calculate the mean of mean_density and exit_flow_rate for each multiplier
    averaged_results = df.groupby('multiplier').agg({
        'mean_density': 'mean',
        'exit_flow_rate': 'mean'
    }).reset_index()

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(averaged_results['mean_density'], averaged_results['exit_flow_rate'], marker='o', linestyle='-')
    plt.xlabel("Mean Density (cars)")
    plt.ylabel("Mean Exit Flow Rate (Cars Exiting per tick)")
    plt.title("Averaged Global Flow Rate vs. Mean Density")
    plt.grid(True)
    plt.xlim(0, 200) # Adjust as needed, matching original plot
    plt.ylim(0, 400) # Adjust as needed, matching original plot

    # Annotate points with multiplier values
    for i, row in averaged_results.iterrows():
        plt.annotate(f'{row["multiplier"]:.2f}', (row['mean_density'], row['exit_flow_rate']), xytext=(5, 5), textcoords='offset points')

    plt.tight_layout()

    plot_filename = "averaged_flow_vs_density_plot.png"
    plt.savefig(plot_filename)
    print(f"Plot saved to {plot_filename}")
    plt.show()

if __name__ == '__main__':
    plot_mean_flow_rate_per_multiplier()
