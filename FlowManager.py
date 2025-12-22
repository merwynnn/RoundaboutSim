import pandas as pd
import numpy as np

class FlowManager:
    SEED = 123
    def __init__(self, config_file='flow_config.xlsx', spawn_intervall_multiplier=1):
        self.flow_matrix = None
        self.flow_rates = None
        self.random_state = np.random.default_rng(self.SEED)

        self.spawn_intervall_multiplier = spawn_intervall_multiplier

        try:
            self.flow_matrix = pd.read_excel(config_file, sheet_name='flow_matrix', index_col=0)
            self.flow_rates = pd.read_excel(config_file, sheet_name='flow_rates', index_col=0)
        except FileNotFoundError:
            print(f"Error: Configuration file '{config_file}' not found.")
        except Exception as e:
            print(f"Error loading configuration file: {e}")

    def set_active_spawners(self, active_spawner_ids):
        if self.flow_matrix is not None:
            # Format IDs to match the Excel file format 'spawner_X'
            formatted_spawner_ids = [f"sp{_id}" for _id in active_spawner_ids]
            
            # Filter columns (starters)
            self.flow_matrix = self.flow_matrix[
                [col for col in self.flow_matrix.columns if col in formatted_spawner_ids]
            ]
            # Filter rows (destinations)
            self.flow_matrix = self.flow_matrix[
                self.flow_matrix.index.isin(formatted_spawner_ids)
            ]

        if self.flow_rates is not None:
            # Filter flow rates
            formatted_spawner_ids = [f"sp{_id}" for _id in active_spawner_ids]
            self.flow_rates = self.flow_rates[
                self.flow_rates.index.isin(formatted_spawner_ids)
            ]

    def get_destination(self, start_extremity_id_int):
        start_extremity_id = f"sp{start_extremity_id_int}"
        if self.flow_matrix is None:
            return None

        if start_extremity_id not in self.flow_matrix.columns:
            print(f"Warning: Start extremity '{start_extremity_id}' not found in flow matrix.")
            return None

        probabilities = self.flow_matrix[start_extremity_id].dropna()

        if probabilities.sum() == 0:
            return None

        probabilities = probabilities / probabilities.sum()
        choices = probabilities.index.to_numpy()
        weights = probabilities.to_numpy()

        destination_id = self.random_state.choice(choices, p=weights)
        #print(int(destination_id.replace('sp', '')))
        return int(destination_id.replace('sp', ''))


    def get_spawn_interval(self, extremity_id_int):
        extremity_id = f"sp{extremity_id_int}"
        if self.flow_rates is None:
            return None
        
        try:
            return self.flow_rates.loc[extremity_id, 'rate']*self.spawn_intervall_multiplier
        except KeyError:
            print(f"Warning: Extremity ID '{extremity_id}' not found in flow rates.")
            return None
