"""Shared publication-quality plot styling for dissertation figures.

Clean MATLAB-style look (boxed axes, light grid, bold titles), a
colour-blind-safe palette (Okabe-Ito), and a save() helper that writes both
a raster PNG (for Word/Docs) and a vector PDF (for LaTeX) of every figure.

    from report_style import apply_style, save, PALETTE
    plt = apply_style()
    fig, ax = plt.subplots()
    ...
    save(fig, 'evidence/output/report/my_figure')   # -> .png AND .pdf
"""

import os

import matplotlib

# Okabe-Ito colour-blind-safe palette
PALETTE = ['#0072B2', '#D55E00', '#009E73', '#CC79A7',
           '#E69F00', '#56B4E9', '#F0E442', '#999999']


def apply_style():
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.figsize': (7.0, 4.3),
        'figure.dpi': 110,
        'savefig.dpi': 220,
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.08,
        'font.family': 'DejaVu Sans',
        'font.size': 11,
        'axes.titlesize': 13,
        'axes.titleweight': 'bold',
        'axes.labelsize': 11.5,
        'axes.labelweight': 'medium',
        'axes.grid': True,
        'axes.axisbelow': True,
        'axes.edgecolor': '#3a3a3a',
        'axes.linewidth': 1.1,
        'axes.prop_cycle': plt.cycler(color=PALETTE),
        'grid.color': '#cfcfcf',
        'grid.linewidth': 0.7,
        'xtick.direction': 'out',
        'ytick.direction': 'out',
        'xtick.color': '#3a3a3a',
        'ytick.color': '#3a3a3a',
        'legend.frameon': True,
        'legend.framealpha': 0.92,
        'legend.edgecolor': '#cccccc',
        'legend.fancybox': False,
    })
    return plt


def save(fig, path_noext, formats=('png', 'pdf')):
    """Saves fig to path_noext.<fmt> for each fmt, then closes it."""
    import matplotlib.pyplot as plt
    d = os.path.dirname(path_noext)
    if d:
        os.makedirs(d, exist_ok=True)
    written = []
    for fmt in formats:
        p = f'{path_noext}.{fmt}'
        fig.savefig(p)
        written.append(p)
    plt.close(fig)
    return written
