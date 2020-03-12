import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from enum import Enum


class Mode(Enum):
    SAFE = 0
    PANIC = 1
    MANUAL = 2
    CALIBRATION = 3
    YAW_CONTROL = 4
    FULL_CONTROL = 5
    RAW = 6
    HEIGHT = 7
    WIRELESS = 8
    LOGGING = 9


def get_data_by_mode(mode, col_names, dataframe):
    if type(col_names) is not list:
        raise ValueError('Argument col_names must be a list')
    col_names.append('Time')
    data = []

    # normalize time
    time_baseline = dataframe.loc[0, 'Time']
    dataframe.loc[:, 'Time'] = dataframe['Time'] - time_baseline

    # divide in groups
    dataframe.loc[:, 'ModeGroup'] = (dataframe['Mode'] != dataframe['Mode'].shift()).cumsum()
    dataframe = dataframe[dataframe['Mode'] == mode.value]
    for _, _df in dataframe.groupby('ModeGroup'):
        _df = _df.loc[:, col_names]
        data.append(_df)

    return data


def plot_from_list_of_datafames(list_of_dataframes):
    n = len(list_of_dataframes)
    fig, axes = plt.subplots(n, figsize=(10, 10))
    labels = []
    handles = []
    for ax, df in zip(axes, list_of_dataframes):
        time = df['Time'] / 1000  # to ms
        df.drop('Time', axis=1, inplace=True)

        for column in df:
            l, = ax.plot(time, df[column])
            ax.set_xlabel('Time (ms)')
            ax.set_ylabel('Value')
            ax.grid(True)

            if len(labels) < len(df.columns):
                labels.append(column)
                handles.append(l)

    plt.figlegend(labels=labels, handles=handles)
    fig.savefig('plots.png')


if __name__ == '__main__':
    df = pd.read_csv("basic_log.txt")
    l_df = get_data_by_mode(Mode.FULL_CONTROL, ['rollSet', 'rollAngle'], df)
    plot_from_list_of_datafames(l_df)
