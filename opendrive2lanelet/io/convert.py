#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""This file is a simple script to convert a xodr file
to a lanelet .xml file."""

import os
import sys
import argparse

# import matplotlib.pyplot as plt
from lxml import etree
from commonroad.scenario.scenario import Scenario

# from commonroad.visualization.draw_dispatch_cr import draw_object

from opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive
from opendrive2lanelet.opendriveparser.parser import parse_opendrive
from opendrive2lanelet.network import Network
from opendrive2lanelet.io.extended_file_writer import ExtendedCommonRoadFileWriter

__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


def parse_arguments():
    parser = argparse.ArgumentParser(description="")
    parser.add_argument("xodr_file", help="xodr file")
    parser.add_argument(
        "-f",
        "--force-overwrite",
        action="store_true",
        help="overwrite existing file if it has same name as converted file",
    )
    parser.add_argument("-o", "--output-name", help="specify name of outputed file")
    args = parser.parse_args()
    return args


# def visualize(scenario: Scenario):
#     """Visualize a created scenario by plotting it
#     with matplotlib.

#     Args:
#       scenario: The scenario which should be plotted.

#     Returns:
#       None

#     """

#     # set plt settings
#     plt.style.use("classic")
#     plt.figure(figsize=(8, 4.5))
#     plt.gca().axis("equal")
#     # plot_limits = [-80, 80, -60, 30]
#     # plot scenario
#     draw_object(scenario)
#     plt.show()


def convert_opendrive(opendrive: OpenDrive) -> Scenario:
    """Convert an existing OpenDrive object to a CommonRoad Scenario.

    Args:
      opendrive: Parsed in OpenDrive map.
    Returns:
      A commonroad scenario with the map represented by lanelets.
    """
    road_network = Network()
    road_network.load_opendrive(opendrive)

    return road_network.export_commonroad_scenario()


def main():
    """Helper function to convert an xodr to a lanelet file
    script uses sys.argv[1] as file to be converted

    """
    args = parse_arguments()

    if args.output_name:
        output_name = args.output_name
    else:
        output_name = args.xodr_file.rpartition(".")[0]
        output_name = f"{output_name}.xml"  # only name of file

    if os.path.isfile(output_name) and not args.force_overwrite:
        print(
            "Not converting because file exists and option 'force-overwrite' not active",
            file=sys.stderr,
        )
        sys.exit(-1)

    with open("{}".format(args.xodr_file), "r") as file_in:
        opendrive = parse_opendrive(etree.parse(file_in).getroot())

    scenario = convert_opendrive(opendrive)

    if not args.osm:
        writer = ExtendedCommonRoadFileWriter(
            scenario, source="OpenDRIVE 2 Lanelet Converter"
        )

        with open(f"{output_name}", "w") as file_out:
            writer.write_scenario_to_file_io(file_out)

    else:
        l2osm = OSMConverter(args.osm)
        osm = l2osm(scenario)
        with open(f"{output_name}", "w") as file_out:
            file_out.write(etree.tostring(osm, encoding="unicode", pretty_print=True))


if __name__ == "__main__":
    main()
