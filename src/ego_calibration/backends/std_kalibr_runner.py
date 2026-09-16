"""Run the delivery solver with a headless matplotlib camera-graph report."""
from __future__ import annotations

import runpy
import sys


def plot_graph(self, fno=0, noShow=True, clearFigure=True, title=''):
    # Kalibr already depends on matplotlib. Drawing directly avoids an optional
    # Cairo dependency and the vendor report's shared /tmp/graph.png file.
    import matplotlib.pyplot as plt
    figure = plt.figure(fno)
    if clearFigure:
        figure.clear()
    figure.suptitle(title)
    axes = figure.add_subplot(111)
    positions = list(self.G.layout('kk'))
    selected = set(getattr(self, 'optimal_baseline_edges', []))
    for edge in self.G.es:
        start, end = positions[edge.source], positions[edge.target]
        axes.plot([start[0], end[0]], [start[1], end[1]], color='#456080',
                  linewidth=4 if edge.index in selected else 1)
        axes.text((start[0]+end[0])/2, (start[1]+end[1])/2, str(edge['weight']),
                  ha='center', va='bottom')
    for vertex, position in zip(self.G.vs, positions):
        axes.text(*position, vertex['label'], ha='center', va='center',
                  bbox={'boxstyle': 'circle,pad=0.8', 'fc': '#cfe5fa', 'ec': '#456080'})
    axes.margins(.3)
    axes.set_aspect('equal', adjustable='datalim')
    axes.axis('off')
    if not noShow:
        plt.show()


def install_report_compatibility():
    import kalibr_camera_calibration as kcc
    import pylab as pl
    kcc.MulticamCalibrationGraph.plotGraphPylab = plot_graph
    original_colorbar = pl.colorbar

    def colorbar(mappable=None, *args, **kwargs):
        # Older Kalibr creates standalone ScalarMappables. Matplotlib >= 3.8
        # needs their target axes explicitly, even when a figure is active.
        if 'ax' not in kwargs and 'cax' not in kwargs and getattr(mappable, 'axes', None) is None:
            kwargs['ax'] = pl.gca()
        return original_colorbar(mappable, *args, **kwargs)

    pl.colorbar = colorbar


def main():
    from kalibr_runner import install_streaming_extractor
    install_report_compatibility()
    install_streaming_extractor()
    sys.argv = sys.argv[1:]
    runpy.run_path(sys.argv[0], run_name='__main__')


if __name__ == '__main__':
    main()
