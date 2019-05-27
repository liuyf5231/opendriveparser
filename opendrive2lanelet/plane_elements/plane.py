# -*- coding: utf-8 -*-

import math
import numpy as np
from numpy.polynomial import polynomial as P
from typing import Tuple

from opendrive2lanelet.lanelet import ConversionLanelet

__author__ = "Benjamin Orthen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class ParametricLaneBorderGroup:
    """Group Borders and BorderOffsets of ParametricLanes into one class.

    Attributes:
      inner_border(Border): Inner Border of a ParametricLane.

      outer_border(Border): Outer Border of a ParametricLane.

      inner_border_offset(float): Offset of start of parametric lane to start
        of border. This is necessary as a Border can be used by multiple ParametricLanes.
        Then each of the ParametricLane defines with inner_border_offset
        where its border starts.

      outer_border_offset(float): Same concept as inner_border_offset, but for outer border.

    """

    # NOTE: not checking types with getter/setter because duck typing
    # should be possible
    def __init__(
        self,
        inner_border=None,
        inner_border_offset=None,
        outer_border=None,
        outer_border_offset=None,
    ):
        self.inner_border = inner_border
        self.inner_border_offset = inner_border_offset
        self.outer_border = outer_border
        self.outer_border_offset = outer_border_offset

    def calc_border_position(
        self, border: str, s_pos: float, width_offset: float, is_last_pos: bool = False
    ) -> Tuple[Tuple[float, float], float]:
        """Calc vertices point of inner or outer Border.

        Args:
          border: Which border to calculate (inner or outer).
          s_pos: Position of parameter ds where to calc the
            Cartesian coordinates
          width_offset: Offset to add to calculated width in reference
            to the reference border.

        Returns:
          Cartesian coordinates of point on inner border
          and tangential direction, too.

        """

        if border not in ("inner", "outer"):
            raise ValueError("Border specified must be 'inner' or 'outer'!")

        select_border = self.inner_border if border == "inner" else self.outer_border
        select_offset = (
            self.inner_border_offset if border == "inner" else self.outer_border_offset
        )

        return select_border.calc(
            select_offset + s_pos, width_offset=width_offset, is_last_pos=is_last_pos
        )

    def get_width_coefficients(self) -> list:
        """Get the width coefficients which apply to this ParametricLane.

        Returns:
          The width coefficients in format [a, b, c, d].
        """
        # TODO: expand implementation to consider border offset record
        return self.outer_border.get_next_width_coeffs(self.outer_border_offset)


class ParametricLane:
    """A lane defines a part of a road along a
    reference trajectory (plan view), using lane borders
    and start/stop positions (parametric)

    Attributes:
      border_group (ParametricLaneBorderGroup): Reference to object which manages borders.
      id_ (str): Unique string identifier.
      type_ (str): Identifies type of ParametricLane.
      length (float): Length of ParametricLane.

    """

    def __init__(
        self,
        id_: str,
        type_: str,
        border_group: ParametricLaneBorderGroup,
        length: float = None,
    ):
        self.border_group = border_group
        self.id_ = id_
        self.type_ = type_
        self.length = length
        self.reverse = False

    def calc_border(
        self, border: str, s_pos: float, width_offset: float = 0.0
    ) -> Tuple[Tuple[float, float], float]:
        """Calc vertices point of inner or outer Border.

        Args:
          border: Which border to calculate (inner or outer).
          s_pos: Position of parameter ds where to calc the
            Cartesian coordinates
          width_offset: Offset to add to calculated width in reference
           to the reference border. (Default value = 0.0)
          is_last_pos: TODO

        Returns:
          Cartesian coordinates of point on inner border
            and tangential direction.

        """
        if self.reverse:
            border_pos = self.length - s_pos
        else:
            border_pos = s_pos

        is_last_pos = np.isclose(self.length, border_pos)

        return self.border_group.calc_border_position(
            border, border_pos, width_offset, is_last_pos
        )

    def calc_width(self, s_pos: float) -> float:
        """Calc width of border at position s_pos.

        Args:
          s_pos: Position of ParametricLane (in curve parameter ds)
            where width should be calculated.

        Returns:
          The width at position s_pos.

        """
        innerCoords = self.calc_border("inner", s_pos)
        outerCoords = self.calc_border("outer", s_pos)

        return np.linalg.norm(innerCoords[0] - outerCoords[0])

    def has_zero_width_everywhere(self) -> bool:
        """Checks if width is zero at every point of this ParametricLaneGroup.

        Returns:
          True if every ParametricLane has width_coefficients equal to only zero.
        """
        # TODO: expand this method to include border offset records
        return self.border_group.get_width_coefficients() == [0, 0, 0, 0]

    def to_lanelet_with_mirroring(
        self,
        mirror_border: str,
        distance: list,
        mirror_interval: list,
        precision: float = 0.5,
    ) -> ConversionLanelet:
        """Convert a ParametricLane to Lanelet.

        Args:
          plane_group: PlaneGroup which should be referenced by created Lanelet.
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.
          mirror_border: Which lane to mirror, if performing merging or splitting of lanes.
          distance: Distance at start and end of lanelet, which mirroring lane should
            have from the other lane it mirrors.

        Returns:
           Created Lanelet, with left, center and right vertices and a lanelet_id.

        """

        num_steps = int(max(3, np.ceil(self.length / float(precision))))

        poses = np.linspace(0, self.length, num_steps)

        left_vertices = []
        right_vertices = []

        # width difference between original_width and width with merge algo applied
        last_width_difference = distance[2]
        distance_slope = (distance[1] - distance[0]) / self.length
        # calculate left and right vertices of lanelet
        for i, pos in enumerate(poses):
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
                t = distance[0]

                d = distance_slope * pos + t

                if mirror_border == "left":
                    new_outer_pos = self.calc_border("inner", pos, d)[0]
                    modified_width = np.linalg.norm(new_outer_pos - inner_pos)

                    # change width s.t. it does not mirror inner border but instead
                    # outer border
                    d = math.copysign(1, d) * last_width_difference
                    if modified_width < original_width:
                        right_vertices.append(self.calc_border("outer", pos, d)[0])
                    else:
                        right_vertices.append(new_outer_pos)
                        last_width_difference = abs(modified_width - original_width)

                    left_vertices.append(inner_pos)
                elif mirror_border == "right":
                    new_inner_pos = self.calc_border("outer", pos, d)[0]
                    modified_width = np.linalg.norm(new_inner_pos - outer_pos)

                    d = math.copysign(1, d) * last_width_difference
                    if modified_width < original_width:
                        left_vertices.append(self.calc_border("inner", pos, d)[0])
                    else:
                        left_vertices.append(new_inner_pos)
                        last_width_difference = abs(modified_width - original_width)

                    right_vertices.append(outer_pos)

        return (
            np.array(left_vertices),
            np.array(right_vertices),
            last_width_difference,
        )

    def calc_vertices(self, precision: float = 0.5) -> Tuple[np.ndarray, np.ndarray]:
        """Convert a ParametricLane to Lanelet.

        Args:
          plane_group: PlaneGroup which should be referenced by created Lanelet.
          precision: Number which indicates at which space interval (in curve parameter ds)
            the coordinates of the boundaries should be calculated.

        Returns:
           Created Lanelet, with left, center and right vertices and a lanelet_id.

        """

        num_steps = int(max(3, np.ceil(self.length / float(precision))))

        poses = np.linspace(0, self.length, num_steps)

        left_vertices = []
        right_vertices = []

        # width difference between original_width and width with merge algo applied

        # calculate left and right vertices of lanelet
        for pos in poses:
            inner_pos = self.calc_border("inner", pos)[0]
            outer_pos = self.calc_border("outer", pos)[0]
            left_vertices.append(inner_pos)
            right_vertices.append(outer_pos)
        return (np.array(left_vertices), np.array(right_vertices))

    def zero_width_change_positions(self) -> float:
        """Position where the inner and outer Border have zero minimal distance change.

        Returns:
          Positions (in curve parameter ds) where width change is zero.
        """

        width_coefficients = self.border_group.get_width_coefficients()
        if width_coefficients[0] > 0.0 and all(
            coeff == 0.0 for coeff in width_coefficients
        ):
            # this is not correct as it should be an interval
            return [0, self.length]
        # but useful because only start and end of ParametricLane should be considered

        # get roots of derivative
        roots = P.polyroots(P.polyder(width_coefficients))
        real_roots = roots[(np.isreal(roots)) & (roots >= 0) & (roots <= self.length)]
        if self.reverse:
            real_roots[:] = [self.length - x for x in real_roots]
        return real_roots

    def maximum_width(self, reverse: bool = False) -> Tuple[float, float]:
        """Get position and value of maximum width.

        Position is the distance of the maximum to the start or end
        of ParametricLane (end if reverse==True).

        Args:
          reverse: If True and there are two equal maxima, take maxima
            nearer to the end of the ParametricLane.
        Returns:
          (pos, max) tuple of position and value of maximum.
        """
        width_coefficients = self.border_group.get_width_coefficients()
        width_derivative = P.polyder(width_coefficients)
        # width_second_derivative = P.polyder(width_derivative)
        roots = P.polyroots(width_derivative)
        # is_local_maximum = P.polyval(roots, width_second_derivative) < 0
        restricted_roots = roots[
            (np.isreal(roots))
            & (roots >= 0)
            # & (is_local_maximum)
            & (roots <= self.length)
        ]

        # append start and end of ParametricLane because maximum could be there, too
        restricted_roots = np.append(restricted_roots, [0, self.length])

        # calculate maximum values
        max_values = P.polyval(restricted_roots, width_coefficients)

        # width of one ParametricLane is either always positive or negative
        max_values = abs(max_values)
        pos_and_val = np.column_stack((restricted_roots, max_values))
        if self.reverse:
            pos_and_val = np.array([[self.length - x[0], x[1]] for x in pos_and_val])

        # sort by position
        if reverse:
            # pos_and_val[:] = pos_and_val[::-1]
            pos_and_val = pos_and_val[pos_and_val[:, 0].argsort()[::-1]]
        else:
            pos_and_val = pos_and_val[pos_and_val[:, 0].argsort()]

        max_idx = np.argmax(pos_and_val, axis=0)[1]
        return tuple(pos_and_val[max_idx])
