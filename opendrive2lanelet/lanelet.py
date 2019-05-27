# -*- coding: utf-8 -*-

"""Module to enhance Lanelet class so it can be used
for conversion from the opendrive format."""


from typing import Tuple

import numpy as np
from commonroad.scenario.lanelet import Lanelet

__author__ = "Benjamin Orthen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"

optimal_join_split_factor = 20


class ConversionLanelet(Lanelet):
    """Change some properties of the Lanelet class so that it can be used
    to convert from OpenDrive to Lanelet. This means especially that lanelet_ids
    can be other types than a natural number and that these ids can be changed
    more than once.

    Args:

    Returns:

    """

    def __init__(
        self,
        parametric_lane_group,
        left_vertices: np.ndarray,
        center_vertices: np.ndarray,
        right_vertices: np.ndarray,
        lanelet_id,
        predecessor=None,
        successor=None,
        adjacent_left=None,
        adjacent_left_same_direction=None,
        adjacent_right=None,
        adjacent_right_same_direction=None,
        speed_limit=np.infty,
        line_marking_left_vertices=None,
        line_marking_right_vertices=None,
    ):
        super().__init__(
            left_vertices,
            center_vertices,
            right_vertices,
            1,
            predecessor,
            successor,
            adjacent_left,
            adjacent_left_same_direction,
            adjacent_right,
            adjacent_right_same_direction,
            speed_limit,
            line_marking_left_vertices,
            line_marking_right_vertices,
        )
        self.parametric_lane_group = parametric_lane_group
        self.lanelet_id = lanelet_id

    def __eq__(self, lanelet: "ConversionLanelet") -> bool:
        """Lanelets are equal if their id_ is equal.

        Args:
           lanelet: Lanelet to be compared to equality.
        Returns:
           True if id_ is equal.
        """
        return self.lanelet_id == lanelet.lanelet_id

    @property
    def lanelet_id(self) -> int:
        """Get or set id of this lanelet."""
        return self._lanelet_id

    @lanelet_id.setter
    def lanelet_id(self, l_id: int):
        # pylint: disable=W0201
        self._lanelet_id = l_id

    @property
    def left_vertices(self) -> np.ndarray:
        """Get or set right vertices of this lanelet."""
        return self._left_vertices

    @left_vertices.setter
    def left_vertices(self, polyline: np.ndarray):
        # pylint: disable=W0201
        self._left_vertices = polyline

    @property
    def right_vertices(self) -> np.ndarray:
        """Get or set right vertices of this lanelet."""
        return self._right_vertices

    @right_vertices.setter
    def right_vertices(self, polyline: np.ndarray):
        # pylint: disable=W0201
        self._right_vertices = polyline

    @property
    def center_vertices(self) -> np.ndarray:
        """Get or set center vertices of this lanelet."""
        return self._center_vertices

    @center_vertices.setter
    def center_vertices(self, polyline: np.ndarray):
        # pylint: disable=W0201
        self._center_vertices = polyline

    @property
    def predecessor(self) -> list:
        """Set or get the predecessor."""
        return self._predecessor

    @predecessor.setter
    def predecessor(self, predecessor: list):
        # pylint: disable=W0201
        self._predecessor = predecessor

    @property
    def successor(self) -> list:
        return self._successor

    @successor.setter
    def successor(self, successor: list):
        # pylint: disable=W0201
        self._successor = successor

    @property
    def adj_left(self) -> int:
        """Set or get adjacent left lanelet."""
        return self._adj_left

    @adj_left.setter
    def adj_left(self, l_id: int):
        # pylint: disable=W0201
        self._adj_left = l_id

    @property
    def adj_left_same_direction(self) -> bool:
        """Set or get if adjacent left lanelet has the same direction
        as this lanelet."""
        return self._adj_left_same_direction

    @adj_left_same_direction.setter
    def adj_left_same_direction(self, same: bool):
        # pylint: disable=W0201
        self._adj_left_same_direction = same

    @property
    def adj_right(self) -> int:
        """Set or get adjacent right lanelet."""
        return self._adj_right

    @adj_right.setter
    def adj_right(self, l_id: int):
        self._adj_right = l_id

    @property
    def adj_right_same_direction(self) -> bool:
        """Set or get if adjacent right lanelet has the same direction
        as this lanelet."""
        return self._adj_right_same_direction

    @adj_right_same_direction.setter
    def adj_right_same_direction(self, same: bool):
        # pylint: disable=W0201
        self._adj_right_same_direction = same

    def concatenate(
        self, lanelet_conc: "ConversionLanelet", extend_plane_group: bool = True
    ) -> "ConversionLanelet":
        """Concatenate this lanelet with lanelet_conc and assign the
        new lanelet_id to the resulting lanelet.

        Args:
          lanelet_conc: Lanelet which will be included.
          extend_plane_group: Whether to extend the parametric_lane_group of this lanelet
        with the parametric lanes of the lanelet_conc.parametric_lane_group.
          lanelet_conc: "ConversionLanelet":
          # lanelet_id: str:  (Default value = -1)
          extend_plane_group: bool:  (Default value = True)

        Returns:

        """

        # float_tolerance = 1e-6
        # if (
        #     np.linalg.norm(self.center_vertices[-1] - lanelet_conc.center_vertices[0])
        #     > float_tolerance
        #     or np.linalg.norm(self.left_vertices[-1] - lanelet_conc.left_vertices[0])
        #     > float_tolerance
        #     or np.linalg.norm(self.right_vertices[-1] - lanelet_conc.right_vertices[0])
        #     > float_tolerance
        # ):
        #     pass
        # raise Exception("no way {} {} {}".format(
        #     np.linalg.norm(self.center_vertices[-1] - lanelet_conc.center_vertices[0]),
        #     np.linalg.norm(self.left_vertices[-1] - lanelet_conc.left_vertices[0]),
        #     np.linalg.norm(self.right_vertices[-1] - lanelet_conc.right_vertices[0])
        # ))
        # return None
        # check connectedness
        if np.isclose(self.left_vertices[-1], lanelet_conc.left_vertices[0]).all():
            idx = 1
        else:
            idx = 0

        self.left_vertices = np.vstack(
            (self.left_vertices, lanelet_conc.left_vertices[idx:])
        )
        self.center_vertices = np.vstack(
            (self.center_vertices, lanelet_conc.center_vertices[idx:])
        )
        self.right_vertices = np.vstack(
            (self.right_vertices, lanelet_conc.right_vertices[idx:])
        )
        if extend_plane_group:
            self.parametric_lane_group.extend(
                lanelet_conc.parametric_lane_group.parametric_lanes
            )
        self.successor = lanelet_conc.successor.copy()

    def calc_width_at_end(self) -> float:
        """Calc width of lanelet at its end.

        Returns:
          Width at end of lanelet.

        """
        return self.calc_width(self.length)

    def calc_width_at_start(self) -> float:
        """Calc width of lanelet at its start.

        Returns:
          Width at start of lanelet.

        """
        return self.calc_width(0)

    def calc_width(self, s_pos: float) -> float:
        """Calc width at position s_pos.

        Args:
          s_pos: Position in curve parameter ds.
        Returns:
          Width at postiion s_pos.
        """
        inner_pos = self.calc_border("inner", s_pos)[0]
        outer_pos = self.calc_border("outer", s_pos)[0]
        return np.linalg.norm(inner_pos - outer_pos)

    @property
    def length(self) -> float:
        """Get length of lanelet by calculating length of ParametricLaneGroup.

        Returns:
          Length of lanelet.
        """
        return self.parametric_lane_group.length

    def has_zero_width_everywhere(self) -> bool:
        """Checks if width is zero at every point of its ParametricLaneGroup.

        Returns:
          True if every ParametricLane has width_coefficients equal to only zero.
        """
        return self.parametric_lane_group.has_zero_width_everywhere()

    def first_zero_width_change_position(
        self, reverse: bool = False, reference_width: float = 0.0
    ) -> float:
        """Get the earliest point of the lanelet where the width change is zero.

        Args:
          reverse: True if checking starts from the end of the lanelet.
          reference_width: Width for which width at zero width change position has
            to be greater as.
        Returns:
          Position of lanelet (in curve parameter ds) where width change is zero.
        """
        return self.parametric_lane_group.first_zero_width_change_position(
            reverse, reference_width
        )

    def maximum_width(self) -> float:
        return self.parametric_lane_group.maximum_width()

    def optimal_join_split_values(
        self, is_split: bool, split_and_join: bool, reference_width: float
    ):
        """Calculate an optimal value, where the lanelet split or join starts
          or ends, respectively.

        Args:
          is_split: True if lanelet splits from another lanelet, otherwise
            False if it is a join.
          split_and_join: True if lanelet has a split at the start and join at the end.
          reference_width: Width for which width at zero width change position has
            to be greater as.
        """

        merge_pos, merge_width = self.first_zero_width_change_position(
            reverse=(not is_split), reference_width=reference_width
        )
        if merge_pos is None:
            merge_pos = self.length if is_split else 0
            # merge_width = self.calc_width(merge_pos)

        if is_split:
            # if both a split at the start and merge at the end
            if split_and_join and merge_pos > 0.5 * self.length:
                merge_pos = 0.45 * self.length

        else:
            if split_and_join and merge_pos < 0.5 * self.length:
                merge_pos = 0.55 * self.length

        # TODO: solve problem with merges which are too long
        # avg_width = 0.5 * (reference_width + self.maximum_width())
        # if is_split and merge_pos > optimal_join_split_factor * avg_width:
        # merge_pos = optimal_join_split_factor * avg_width
        # elif merge_pos < self.length - optimal_join_split_factor * avg_width:
        # merge_pos = self.length - optimal_join_split_factor * avg_width
        merge_width = self.calc_width(merge_pos)
        return merge_pos, merge_width

    def move_border(
        self,
        mirror_border: str,
        mirror_interval: Tuple[float, float],
        distance: Tuple[float, float],
        adjacent_lanelet: "ConversionLanelet",
    ):
        """Move vertices of one border by mirroring other border with
        a specified distance.

        Args:
          mirror_border: Which border to mirror, either 'left' or 'right'.
          interval: Tuple of two values, specifying start and end of mirroring.
          distance: Specifying distance at start and at end of mirroring.
        """
        if mirror_border == "left":
            distance[:] = [-1 * x for x in distance]

        lanelet = self.parametric_lane_group.to_lanelet_with_mirroring(
            mirror_border=mirror_border,
            distance=distance,
            mirror_interval=mirror_interval,
            adjacent_lanelet=adjacent_lanelet,
        )

        self.left_vertices = lanelet.left_vertices
        self.center_vertices = lanelet.center_vertices
        self.right_vertices = lanelet.right_vertices

    def calc_border(self, border: str, s_pos: float, width_offset: float = 0.0):
        """Calc border position according to parametric_lane_group.

        Note: This does not consider borders which have been moved
         due to joining / splitting.

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
        return self.parametric_lane_group.calc_border(border, s_pos, width_offset)
