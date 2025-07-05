import pandas as pd
import numpy as np

class FlowManager:
    SEED = 123
    def __init__(self, config_file='flow_config.xlsx'):
        self.flow_matrix = None
        self.flow_rates = None
        self.random_state = np.random.RandomState(self.SEED) if self.SEED is not None else None
        try:
            self.flow_matrix = pd.read_excel(config_file, sheet_name='flow_matrix', index_col=0)
            self.flow_rates = pd.read_excel(config_file, sheet_name='flow_rates', index_col=0)
        except FileNotFoundError:
            print(f"Error: Configuration file '{config_file}' not found.")
        except Exception as e:
            print(f"Error loading configuration file: {e}")

    def get_destination(self, start_extremity_id_int):
        start_extremity_id = f"spawner_{start_extremity_id_int}"
        if self.flow_matrix is None:
            return None
        
        if start_extremity_id not in self.flow_matrix.columns:
            print(f"Warning: Start extremity '{start_extremity_id}' not found in flow matrix.")
            return None

        probabilities = self.flow_matrix[start_extremity_id]
        # Drop NA values to avoid issues with the choice function
        probabilities = probabilities.dropna()
        if probabilities.sum() == 0:
            return None
        
        # Normalize probabilities to ensure they sum to 1
        probabilities = probabilities / probabilities.sum()
        
        destination_id = probabilities.sample(n=1, weights=probabilities, random_state=self.random_state).index[0]
        return int(destination_id.replace('spawner_', ''))

    def get_spawn_interval(self, extremity_id_int):
        extremity_id = f"spawner_{extremity_id_int}"
        if self.flow_rates is None:
            return None
        
        try:
            return self.flow_rates.loc[extremity_id, 'spawn_interval']
        except KeyError:
            print(f"Warning: Extremity ID '{extremity_id}' not found in flow rates.")
            return None
