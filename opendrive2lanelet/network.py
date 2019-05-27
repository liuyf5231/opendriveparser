# -*- coding: utf-8 -*-


"""Module to contain Network which can load an opendrive object and then export
to lanelets. Iternally, the road network is represented by ParametricLanes."""

from commonroad.scenario.scenario import Scenario

from opendrive2lanelet.opendriveparser.elements.opendrive import OpenDrive

from opendrive2lanelet.utils import encode_road_section_lane_width_id
from opendrive2lanelet.lanelet_network import ConversionLaneletNetwork
from opendrive2lanelet.converter import OpenDriveConverter


__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


def convert_to_new_lanelet_id(old_lanelet_id: str, ids_assigned: dict) -> int:
    """Convert the old lanelet ids (format 501.1.-1.-1) to newer,
    simpler ones (100, 101 etc.).

    Do this by consecutively assigning
    numbers, starting at 100, to the old_lanelet_id strings. Save the
    assignments in the dict which is passed to the function as ids_assigned.

    Args:
      old_lanelet_id: Old id with format "501.1.-1.-1".
      ids_assigned: Dict with all previous assignments

    Returns:
      The new lanelet id.

    """

    starting_lanelet_id = 100

    if old_lanelet_id in ids_assigned.keys():
        new_lanelet_id = ids_assigned[old_lanelet_id]
    else:
        try:
            new_lanelet_id = max(ids_assigned.values()) + 1
        except ValueError:
            new_lanelet_id = starting_lanelet_id
        ids_assigned[old_lanelet_id] = new_lanelet_id

    return new_lanelet_id


class Network:
    """Represents a network of parametric lanes, with a LinkIndex
    which stores the neighbor relations between the parametric lanes.

    Args:

    """

    def __init__(self):
        self._planes = []
        self._link_index = None

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def load_opendrive(self, opendrive: OpenDrive):
        """Load all elements of an OpenDRIVE network to a parametric lane representation

        Args:
          opendrive:

        """

        if not isinstance(opendrive, OpenDrive):
            raise TypeError()

        self._link_index = LinkIndex()
        self._link_index.create_from_opendrive(opendrive)

        # Convert all parts of a road to parametric lanes (planes)
        for road in opendrive.roads:
            road.planView.precalculate()

            # The reference border is the base line for the whole road
            reference_border = OpenDriveConverter.create_reference_border(
                road.planView, road.lanes.laneOffsets
            )

            # A lane section is the smallest part that can be converted at once
            for lane_section in road.lanes.lane_sections:

                parametric_lane_groups = OpenDriveConverter.lane_section_to_parametric_lanes(
                    lane_section, reference_border
                )

                self._planes.extend(parametric_lane_groups)

    def export_lanelet_network(
        self, filter_types: list = None
    ) -> "ConversionLaneletNetwork":
        """Export network as lanelet network.

        Args:
          filter_types: types of ParametricLane objects to be filtered. (Default value = None)

        Returns:
          The converted LaneletNetwork object.
        """

        # Convert groups to lanelets
        lanelet_network = ConversionLaneletNetwork()

        for parametric_lane in self._planes:
            if filter_types is not None and parametric_lane.type not in filter_types:
                continue

            lanelet = parametric_lane.to_lanelet()

            lanelet.predecessor = self._link_index.get_predecessors(parametric_lane.id_)
            lanelet.successor = self._link_index.get_successors(parametric_lane.id_)

            lanelet_network.add_lanelet(lanelet)

        # prune because some
        # successorIds get encoded with a non existing successorID
        # of the lane link
        lanelet_network.prune_network()
        lanelet_network.concatenate_possible_lanelets()

        # Perform lane splits and joins
        # Condition for lane merge:
        # - Lanelet ends (has no successor or predecessor)
        # - Has an adjacent (left or right) with same type
        lanelet_network.join_and_split_possible_lanes()

        # dict where the connections between the old and new
        # ids are saved
        new_lanelet_ids_assigned = {}

        for lanelet in lanelet_network.lanelets:
            lanelet.description = lanelet.lanelet_id
            lanelet_network.remove_lanelet(lanelet.lanelet_id)
            lanelet.lanelet_id = convert_to_new_lanelet_id(
                lanelet.lanelet_id, new_lanelet_ids_assigned
            )
            lanelet.predecessor = [
                convert_to_new_lanelet_id(x, new_lanelet_ids_assigned)
                for x in lanelet.predecessor
            ]
            lanelet.successor = [
                convert_to_new_lanelet_id(x, new_lanelet_ids_assigned)
                for x in lanelet.successor
            ]
            if lanelet.adj_left is not None:
                lanelet.adj_left = convert_to_new_lanelet_id(
                    lanelet.adj_left, new_lanelet_ids_assigned
                )
            if lanelet.adj_right is not None:
                lanelet.adj_right = convert_to_new_lanelet_id(
                    lanelet.adj_right, new_lanelet_ids_assigned
                )
            lanelet_network.add_lanelet(lanelet)

        return lanelet_network

    def export_commonroad_scenario(
        self, dt: float = 0.1, benchmark_id=None, filter_types=None
    ):
        """Export a full CommonRoad scenario

        Args:
          dt:  (Default value = 0.1)
          benchmark_id:  (Default value = None)
          filter_types:  (Default value = None)

        Returns:

        """

        scenario = Scenario(
            dt=dt, benchmark_id=benchmark_id if benchmark_id is not None else "none"
        )

        scenario.add_objects(
            self.export_lanelet_network(
                filter_types=filter_types
                if isinstance(filter_types, list)
                else ["driving", "onRamp", "offRamp", "exit", "entry"]
            )
        )

        return scenario


class LinkIndex:
    """Overall index of all links in the file, save everything as successors, predecessors can be found via a reverse search"""

    def __init__(self):
        self._successors = {}

    def create_from_opendrive(self, opendrive):
        """Create a LinkIndex from an OpenDrive object.

        Args:
          opendrive: OpenDrive style object.

        Returns:

        """
        self._add_junctions(opendrive)
        # Extract link information from road lanes
        for road in opendrive.roads:
            for lane_section in road.lanes.lane_sections:
                for lane in lane_section.allLanes:
                    parametric_lane_id = encode_road_section_lane_width_id(
                        road.id, lane_section.idx, lane.id, -1
                    )

                    # Not the last lane section? > Next lane section in same road
                    if lane_section.idx < road.lanes.getLastLaneSectionIdx():
                        successorId = encode_road_section_lane_width_id(
                            road.id, lane_section.idx + 1, lane.link.successorId, -1
                        )

                        self.add_link(parametric_lane_id, successorId, lane.id >= 0)

                    # Last lane section! > Next road in first lane section
                    # Try to get next road
                    elif (
                        road.link.successor is not None
                        and road.link.successor.elementType != "junction"
                    ):

                        next_road = opendrive.getRoad(road.link.successor.element_id)

                        if next_road is not None:

                            if road.link.successor.contactPoint == "start":
                                successorId = encode_road_section_lane_width_id(
                                    next_road.id, 0, lane.link.successorId, -1
                                )

                            else:  # contact point = end
                                successorId = encode_road_section_lane_width_id(
                                    next_road.id,
                                    next_road.lanes.getLastLaneSectionIdx(),
                                    lane.link.successorId,
                                    -1,
                                )
                            self.add_link(parametric_lane_id, successorId, lane.id >= 0)

                    # Not first lane section? > Previous lane section in same road
                    if lane_section.idx > 0:
                        predecessorId = encode_road_section_lane_width_id(
                            road.id, lane_section.idx - 1, lane.link.predecessorId, -1
                        )

                        self.add_link(predecessorId, parametric_lane_id, lane.id >= 0)

                    # First lane section! > Previous road
                    # Try to get previous road
                    elif (
                        road.link.predecessor is not None
                        and road.link.predecessor.elementType != "junction"
                    ):

                        prevRoad = opendrive.getRoad(road.link.predecessor.element_id)

                        if prevRoad is not None:

                            if road.link.predecessor.contactPoint == "start":
                                predecessorId = encode_road_section_lane_width_id(
                                    prevRoad.id, 0, lane.link.predecessorId, -1
                                )

                            else:  # contact point = end
                                predecessorId = encode_road_section_lane_width_id(
                                    prevRoad.id,
                                    prevRoad.lanes.getLastLaneSectionIdx(),
                                    lane.link.predecessorId,
                                    -1,
                                )
                            self.add_link(
                                predecessorId, parametric_lane_id, lane.id >= 0
                            )

    def add_link(self, parametric_lane_id, successor, reverse: bool = False):
        """

        Args:
          parametric_lane_id:
          successor:
          reverse:  (Default value = False)

        Returns:

        """

        # if reverse, call function recursively with switched parameters
        if reverse:
            self.add_link(successor, parametric_lane_id)
            return

        if parametric_lane_id not in self._successors:
            self._successors[parametric_lane_id] = []

        if successor not in self._successors[parametric_lane_id]:
            self._successors[parametric_lane_id].append(successor)

    def _add_junctions(self, opendrive):
        """

        Args:
          opendrive:

        Returns:

        """
        # add junctions to link_index
        # if contact_point is start, and laneId from connecting_road is negative
        # the connecting_road is the successor
        # if contact_point is start, and laneId from connecting_road is positive
        # the connecting_road is the predecessor
        # for contact_point == end it's exactly the other way
        for junction in opendrive.junctions:
            for connection in junction.connections:
                incoming_road = opendrive.getRoad(connection.incomingRoad)
                connecting_road = opendrive.getRoad(connection.connectingRoad)
                contact_point = connection.contactPoint

                for lane_link in connection.laneLinks:
                    if contact_point == "start":

                        # decide which lane section to use (first or last)
                        if lane_link.fromId < 0:
                            lane_section_idx = (
                                incoming_road.lanes.getLastLaneSectionIdx()
                            )
                        else:
                            lane_section_idx = 0
                        incoming_road_id = encode_road_section_lane_width_id(
                            incoming_road.id, lane_section_idx, lane_link.fromId, -1
                        )
                        connecting_road_id = encode_road_section_lane_width_id(
                            connecting_road.id, 0, lane_link.toId, -1
                        )
                        self.add_link(
                            incoming_road_id, connecting_road_id, lane_link.toId > 0
                        )
                    else:
                        # decide which lane section to use (first or last)
                        if lane_link.fromId < 0:
                            lane_section_idx = 0

                        else:
                            lane_section_idx = (
                                incoming_road.lanes.getLastLaneSectionIdx()
                            )
                        incoming_road_id = encode_road_section_lane_width_id(
                            incoming_road.id, 0, lane_link.fromId, -1
                        )
                        connecting_road_id = encode_road_section_lane_width_id(
                            connecting_road.id,
                            connecting_road.lanes.getLastLaneSectionIdx(),
                            lane_link.toId,
                            -1,
                        )
                        self.add_link(
                            incoming_road_id, connecting_road_id, lane_link.toId < 0
                        )

    def remove(self, parametric_lane_id):
        """

        Args:
          parametric_lane_id:

        """
        # Delete key
        if parametric_lane_id in self._successors:
            del self._successors[parametric_lane_id]

        # Delete all occurances in successor lists
        for successorsId, successors in self._successors.items():
            if parametric_lane_id in successors:
                self._successors[successorsId].remove(parametric_lane_id)

    def get_successors(self, parametric_lane_id: str) -> list:
        """

        Args:
          parametric_lane_id: Id of ParametricLane for which to search
            successors.

        Returns:
          List of successors belonging the the ParametricLane.
        Par
        """
        if parametric_lane_id not in self._successors:
            return []

        return self._successors[parametric_lane_id]

    def get_predecessors(self, parametric_lane_id: str) -> list:
        """

        Args:
          parametric_lane_id: Id of ParametricLane for which to search predecessors.

        Returns:
          List of predecessors of a ParametricLane.
        """
        predecessors = []
        for successor_plane_id, successors in self._successors.items():
            if parametric_lane_id not in successors:
                continue

            if successor_plane_id in predecessors:
                continue

            predecessors.append(successor_plane_id)

        return predecessors
