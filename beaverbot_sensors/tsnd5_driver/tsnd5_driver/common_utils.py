# flake8: noqa
# reference: https://github.com/maselab/tsnd
############ common utilities #############
def check_range(val_name, val, min_, max_):
    if val < min_ or max_ < val:
        raise ValueError("Invalid '{}': range is [{},{}] but {}".format(val_name, min_, max_, val))


def resample_df(df, ix_to):
    """Resampling a df with the new ix

    Extended description of function.

    Parameters
    ----------
    df:
        pandas.DataFrame
    ix_to:
        requirement: type(ix_to) == type(df.index)

    Returns
    -------
    df
        resampled pandas.DataFrame
    """
    import pandas as pd
    import numpy as np
    ix_to = ix_to[[not b for b in np.in1d(ix_to, df.index)]]  # remove duplicated index
    tmp_df = df.append(pd.DataFrame(np.repeat(np.NaN, len(ix_to)), index=ix_to))
    return tmp_df.sort_index().interpolate('index').loc[ix_to]