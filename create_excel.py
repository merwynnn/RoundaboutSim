import pandas as pd

# Create a sample flow matrix for a 2x2 grid (8 spawners)
spawner_ids = [f'spawner_{i}' for i in range(8)]
flow_matrix_data = {start: [1/7 if start != end else 0 for end in spawner_ids] for start in spawner_ids}
flow_matrix_df = pd.DataFrame(flow_matrix_data, index=spawner_ids)

# Create a sample flow rates
flow_rates_data = {'spawn_interval': [120, 120, 240, 240, 120, 120, 240, 240]}
flow_rates_df = pd.DataFrame(flow_rates_data, index=spawner_ids)
flow_rates_df.index.name = 'extremity_id'


# Create an Excel writer
with pd.ExcelWriter('flow_config.xlsx') as writer:
    flow_matrix_df.to_excel(writer, sheet_name='flow_matrix')
    flow_rates_df.to_excel(writer, sheet_name='flow_rates')

print("Excel file created successfully.")
