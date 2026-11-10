"""Load a tracked laser-dot series and drop detection glitches."""
import numpy as np
from scipy.ndimage import median_filter


def load(f):
    """Return t (s), x, y (px, raw sensor axes) for track t<f>.npy, with glitches interpolated over."""
    d = np.load(f"t{f}.npy"); t = d[:, 0]/240; x = d[:, 1].copy(); y = d[:, 2].copy()
    bad = np.isnan(y) | (d[:, 3] < 20)                     # too few saturated core pixels
    xm = median_filter(np.nan_to_num(x), 481); bad |= np.abs(x - xm) > 12   # jumped to another red object
    yi = np.interp(t, t[~bad], y[~bad]); bad |= np.abs(yi - median_filter(yi, 9)) > 3
    return t, np.interp(t, t[~bad], x[~bad]), np.interp(t, t[~bad], y[~bad])
