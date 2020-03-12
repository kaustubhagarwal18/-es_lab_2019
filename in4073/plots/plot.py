from .processing import plot_from_list_of_datafames, get_data_by_mode, Mode
import pandas as pd

df = pd.read_csv("basic_log.txt")

list_of_df = get_data_by_mode(mode=Mode.FULL_CONTROL, col_names=['rollSet', 'rollAngle'], dataframe=df)

plot_from_list_of_datafames(list_of_df)
