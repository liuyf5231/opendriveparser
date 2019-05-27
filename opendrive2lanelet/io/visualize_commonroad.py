# -*- coding: utf-8 -*-

import argparse
import matplotlib.pyplot as plt

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.visualization.draw_dispatch_cr import draw_object


def parse_arguments():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("xml_file", help="CommonRoad XML file")
    parser.add_argument(
        "-s", "--show-labels", action="store_true", help="show labels of lanelets"
    )
    # TODO: add plot center as argument
    args = parser.parse_args()
    return args


def main():
    """Short helper file to visualize an xml file
    as a command line tool.

    """

    args = parse_arguments()

    filename = args.xml_file

    scenario, _ = CommonRoadFileReader(filename).open()

    draw_params = {
        "scenario": {"lanelet_network": {"lanelet": {"show_label": args.show_labels}}}
    }
    # temporary fix to get a plotable view of the scenario
    plot_center = scenario.lanelet_network.lanelets[0].left_vertices[0]
    plt.style.use("classic")
    plt.figure(figsize=(10, 10))
    plt.gca().axis("equal")
    plot_displacement_x = plot_displacement_y = 200
    plot_limits = [
        plot_center[0] - plot_displacement_x,
        plot_center[0] + plot_displacement_x,
        plot_center[1] - plot_displacement_y,
        plot_center[1] + plot_displacement_y,
    ]
    draw_object(scenario, plot_limits=plot_limits, draw_params=draw_params)
    plt.show()


if __name__ == "__main":
    main()
