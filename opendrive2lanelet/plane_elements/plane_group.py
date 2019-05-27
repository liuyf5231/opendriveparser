# -*- coding: utf-8 -*-

from typing import Tuple
import math
import numpy as np
from opendrive2lanelet.lanelet import ConversionLanelet

__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class ParametricLaneGroup:
    """A group of parametric_lanes can be converted to a
    lanelet just like a single parametric_lane.
    """

    def __init__(
        self,
        id_=None,
        parametric_lanes=None,
        inner_neighbour=None,
        inner_neighbour_same_direction=True,
        outer_neighbour=None,
    ):

        self._geo_lengths = [np.array([0.0])]
        self.parametric_lanes = []
        self.id_ = id_
        self.inner_neighbour = inner_neighbour
        self.inner_neighbour_same_direction = inner_neighbour_same_direction
        self.outer_neighbour = outer_neighbour

        if parametric_lanes is not None:
            if isinstance(parametric_lanes, list):
                self.extend(parametric_lanes)
            else:
                self.append(parametric_lanes)

    def append(self, parametric_lane):
        """Append lane to start or end of interal list of ParametricLane objects.

        If the parametric_lane is reverse, it is inserted at the start.
        Args:
          parametric_lane: Lane to be inserted either at beginning or end of list.
        """
        if parametric_lane.reverse:
            self.parametric_lanes.insert(0, parametric_lane)
        else:
            self.parametric_lanes.append(parametric_lane)

        self._add_geo_length(parametric_lane.length, parametric_lane.reverse)

    def extend(self, plane_list: list):
        """Extend own ParametricLanes with new ones.

        Assumes ParametricLane objects in plane_list are already in order.

        Args:
          plane_list: List with ParametricLane objects.
        """
        for plane in plane_list:
            self.parametric_lanes.append(plane)
            self._add_geo_length(plane.length)

    def _add_geo_length(self, length: float, reverse: bool = False):
        """Add length of a ParametricLane to the array which keeps track
        at which position which ParametricLane is placed.

        This array is used for quickly accessing
        the proper ParametricLane for calculating a position.

        Args:
          length: Length of ParametricLane to be added.

        """
        if reverse:
            self._geo_lengths = np.insert(self._geo_lengths, 1, length)
            self._geo_lengths[2:] = [
                x + length for i, x in enumerate(self._geo_lengths) if i > 1
            ]
        else:
            self._geo_lengths = np.append(
                self._geo_lengths, length + self._geo_lengths[-1]
            )

    @property
    def type(self) -> str:
        """Get type of ParametricLaneGroup.


        Returns:
          Type of first ParametricLane in this Group.
        """
        return self.parametric_lanes[0].type_

    @property
    def length(self) -> float:
        """Length of all ParametricLanes which are collected in this ParametricLaneGroup.

        Returns:
          Accumulated length of ParametricLaneGroup.
        """

        return sum([x.length for x in self.parametric_lanes])

    def has_zero_width_everywhere(self) -> bool:
        """Checks if width is zero at every point of this ParametricLaneGroup.

        Returns:
          True if every ParametricLane has width_coefficients equal to only zero.
        """
        return all(
            [plane.has_zero_width_everywhere() for plane in self.parametric_lanes]
        )

    def to_lanelet(self, precision: float = 0.5) -> ConversionLanelet:
        """Convert a ParametricLaneGroup to a Lanelet.

        Args:
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.
          mirror_border: Which lane to mirror, if performing merging or splitting of lanes.
          distance: Distance at start and end of lanelet, which mirroring lane should
            have from the other lane it mirrors.

        Returns:
          Created Lanelet.

        """
        left_vertices, right_vertices = np.array([]), np.array([])

        for parametric_lane in self.parametric_lanes:

            local_left_vertices, local_right_vertices = parametric_lane.calc_vertices(
                precision=precision
            )

            if local_left_vertices is None:
                continue

            try:
                if np.isclose(left_vertices[-1], local_left_vertices[0]).all():
                    idx = 1
                else:
                    idx = 0
                left_vertices = np.vstack((left_vertices, local_left_vertices[idx:]))
                right_vertices = np.vstack((right_vertices, local_right_vertices[idx:]))
            except IndexError:
                left_vertices = local_left_vertices
                right_vertices = local_right_vertices

        center_vertices = np.array(
            [(l + r) / 2 for (l, r) in zip(left_vertices, right_vertices)]
        )

        lanelet = ConversionLanelet(
            self, left_vertices, center_vertices, right_vertices, self.id_
        )

        # Adjacent lanes
        self._set_adjacent_lanes(lanelet)

        return lanelet

    def calc_border(self, border: str, s_pos: float, width_offset: float = 0.0):
        """Calc vertices point of inner or outer Border.

        Args:
          border: Which border to calculate (inner or outer).
          s_pos: Position of parameter ds where to calc the
        Cartesian coordinates
          width_offset: Offset to add to calculated width in reference
           to the reference border. (Default value = 0.0)

        Returns:
          Cartesian coordinates of point on inner border
            and tangential direction, too.

        """
        try:
            # get index of geometry which is at s_pos
            mask = self._geo_lengths > s_pos
            sub_idx = np.argmin(self._geo_lengths[mask] - s_pos)
            plane_idx = np.arange(self._geo_lengths.shape[0])[mask][sub_idx] - 1
        except ValueError:
            # s_pos is after last geometry because of rounding error
            if np.isclose(s_pos, self._geo_lengths[-1]):
                plane_idx = self._geo_lengths.size - 2
            else:
                raise Exception(
                    f"Tried to calculate a position outside of the borders of the reference path at s={s_pos}"
                    f", but path has only length of l={ self._geo_lengths[-1]}"
                )

        return self.parametric_lanes[plane_idx].calc_border(
            border, s_pos - self._geo_lengths[plane_idx], width_offset
        )

    def to_lanelet_with_mirroring(
        self,
        mirror_border: str,
        distance: Tuple[float, float],
        mirror_interval: Tuple[float, float],
        adjacent_lanelet: ConversionLanelet,
        precision: float = 0.5,
    ):
        """Convert a ParametricLaneGroup to a Lanelet with mirroring one of the borders.

        Args:
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.
          mirror_border: Which lane to mirror, if performing merging or splitting of lanes.
          distance: Distance at start and end of lanelet, which mirroring lane should
            have from the other lane it mirrors.
          mirror_interval: Position at start and end of mirroring.

        Returns:
          Created Lanelet.

        """
        linear_distance_poly = np.polyfit(mirror_interval, distance, 1)
        distance_poly1d = np.poly1d(linear_distance_poly)
        global_distance = distance_poly1d([0, self.length])

        if self.parametric_lanes[0].reverse:
            global_distance[:] = [-x for x in global_distance]
        left_vertices, right_vertices = [], []

        last_width_difference = 0
        poses = self._calc_border_positions(precision)
        distance_slope = (global_distance[1] - global_distance[0]) / self.length
        for pos in poses:
            inner_pos = self.calc_border("inner", pos)[0]
            outer_pos = self.calc_border("outer", pos)[0]
            original_width = np.linalg.norm(inner_pos - outer_pos)

            # if not mirroring lane or outside of range
            if (
                pos < mirror_interval[0] or pos > mirror_interval[1]
            ) and not np.isclose(pos, mirror_interval[1]):
                left_vertices.append(inner_pos)
                right_vertices.append(outer_pos)
                last_width_difference = 0

            else:
                # calculate positions of adjacent lanelet because new width of lanelet
                # cannot be more than width of adjacent lanelet and original width
                adj_inner_pos = adjacent_lanelet.calc_border("inner", pos)[0]
                adj_outer_pos = adjacent_lanelet.calc_border("outer", pos)[0]
                adjacent_width = np.linalg.norm(adj_inner_pos - adj_outer_pos)
                local_width_offset = distance_slope * pos + global_distance[0]

                if mirror_border == "left":
                    new_outer_pos = self.calc_border("inner", pos, local_width_offset)[
                        0
                    ]
                    modified_width = np.linalg.norm(new_outer_pos - inner_pos)

                    # change width s.t. it does not mirror inner border but instead
                    # outer border
                    local_width_offset = (
                        math.copysign(1, local_width_offset) * last_width_difference
                    )
                    if modified_width < original_width:
                        right_vertices.append(
                            self.calc_border("outer", pos, local_width_offset)[0]
                        )
                    elif modified_width > original_width + adjacent_width:
                        right_vertices.append(adj_outer_pos)
                    else:
                        right_vertices.append(new_outer_pos)
                        last_width_difference = abs(modified_width - original_width)

                    left_vertices.append(inner_pos)
                elif mirror_border == "right":
                    new_inner_pos = self.calc_border("outer", pos, local_width_offset)[
                        0
                    ]
                    modified_width = np.linalg.norm(new_inner_pos - outer_pos)

                    local_width_offset = (
                        math.copysign(1, local_width_offset) * last_width_difference
                    )
                    if modified_width < original_width:
                        left_vertices.append(
                            self.calc_border("inner", pos, local_width_offset)[0]
                        )
                    elif modified_width > original_width + adjacent_width:
                        left_vertices.append(adj_inner_pos)
                    else:
                        left_vertices.append(new_inner_pos)
                        last_width_difference = abs(modified_width - original_width)

                    right_vertices.append(outer_pos)

        left_vertices = np.array(left_vertices)
        right_vertices = np.array(right_vertices)

        center_vertices = np.array(
            [(l + r) / 2 for (l, r) in zip(left_vertices, right_vertices)]
        )
        lanelet = ConversionLanelet(
            self, left_vertices, center_vertices, right_vertices, self.id_
        )

        # Adjacent lanes
        self._set_adjacent_lanes(lanelet)

        return lanelet

    def _calc_border_positions(self, precision: float) -> np.ndarray:
        """Determine the positions along the border where the coordinates
        of the border should be calculated.

        Args:
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.

        Returns:
          Array with the ordered positions.
        """
        poses = np.array([])
        for i, parametric_lane in enumerate(self.parametric_lanes):
            num_steps = int(max(2, np.ceil(parametric_lane.length / float(precision))))
            if not i:
                idx = 0
            else:
                idx = 1

            poses = np.append(
                poses,
                np.linspace(0, parametric_lane.length, num_steps)[idx::]
                + self._geo_lengths[i],
            )

        return poses

    def _set_adjacent_lanes(self, lanelet: ConversionLanelet):
        """While converting a ParametricLaneGroup to a Lanelet, set
        the proper attributes relating to adjacent lanes.

        Args:
          lanelet: The lanelet which is created from the ParametricLaneGroup.
        """
        if self.inner_neighbour is not None:
            lanelet.adj_left = self.inner_neighbour
            lanelet.adj_left_same_direction = self.inner_neighbour_same_direction

        if self.outer_neighbour is not None:
            lanelet.adj_right = self.outer_neighbour
            lanelet.adj_right_same_direction = True

    def maximum_width(self) -> float:
        """Get the maximum width of the lanelet.

        Returns:
          Maximum width of all ParametricLanes in this Group.
        """
        total_maximum = 0

        for plane in self.parametric_lanes:
            _, maximum = plane.maximum_width()
            if maximum > total_maximum:
                total_maximum = maximum

        return total_maximum

    def first_zero_width_change_position(
        self, reverse: bool = False, reference_width: float = 0.0
    ) -> float:
        """Get the earliest point of the ParametricLaneGroup where the width change is zero.

        Args:
          reverse: True if checking should start from end of lanelet.
          reference_width: Width for which width at zero width change position has
            to be greater as.

        Returns:
          Position of ParametricLaneGroup (in curve parameter ds) where width change is zero.
        """
        s_pos = 0
        positions = []

        # total_maximum = self.maximum_width()

        for plane in self.parametric_lanes:
            zero_change_positions = plane.zero_width_change_positions()
            for pos in zero_change_positions:
                positions.append((pos + s_pos, plane.calc_width(pos)))
            s_pos += plane.length

        if reverse:
            positions = list(reversed(positions))

        # if lanelet has zero width change and its width
        # is either near the maximum or it is greater than the reference width
        # the position can be used
        for pos, val in positions:
            if val > 0.9 * reference_width or val > 0.9 * self.maximum_width():
                if (pos == 0.0 and not reverse) or (pos == self.length and reverse):
                    continue
                return pos, val

        return None, None

    # FIXME: unused, candidate for deletion
    def first_width_change_reversal_point(self, reverse: bool = False) -> float:
        """Get the first point where width change is zero or point is between two ParametricLanes.

        This method considers that not all positions where the derivative of the width
        (which is a polynomial in s) is a position where the width derivative is zero.
        The other option is that between two ParametricLanes of this Group the width
        derivative changes not constant.

        Args:
          reverse: True if checking should start from end of lanelet.
        Returns:
          Position of ParametricLaneGroup (in curve parameter ds) where width derivate
            changes from positive to negative.
        """

        width_derivative_zero = self.first_zero_width_change_position(reverse)
        between_plane_width_dev_changes = [width_derivative_zero]
        for i in range(len(self.parametric_lanes) - 1):
            first_plane = self.parametric_lanes[i]
            second_plane = self.parametric_lanes[i + 1]
            if first_plane.calc_width(
                0.99 * first_plane.length
            ) > second_plane.calc_width(0 + 0.01 * second_plane.length):
                between_plane_width_dev_changes.append(
                    first_plane.length + self._geo_lengths[i]
                )

        return next(sorted(between_plane_width_dev_changes))
