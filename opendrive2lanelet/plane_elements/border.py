# -*- coding: utf-8 -*-

from functools import lru_cache
import numpy as np

__author__ = "Benjamin Orthen, Stefan Urban"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class Border:
    """A lane border defines a path along a whole lane section
    - a lane always uses an inner and outer lane border
    - the reference can be another lane border or a plan view

    Attributes:


    Args:

    """

    def __init__(self, ref_offset: float = 0.0):

        self.ref_offset = float(ref_offset)
        self.width_coefficient_offsets = []
        self.width_coefficients = []

        self.reference = None

    def _get_width_index(self, s_pos: float, is_last_pos: bool) -> int:
        """Get the index of the width which applies at position s_pos.

        Args:
          s_pos: Position on border in curve_parameter ds.
        Returns:
          Index for self.width_coefficient_offsets or self.width_coefficients.
        """
        return next(
            (
                self.width_coefficient_offsets.index(n)
                for n in self.width_coefficient_offsets[::-1]
                if (
                    (n <= s_pos and (not is_last_pos or s_pos == 0))
                    or (n < s_pos and is_last_pos)
                )
            ),
            len(self.width_coefficient_offsets),
        )

    def get_next_width_coeffs(self, s_pos: float, is_last_pos: bool = False) -> list:
        """Get width coefficients which apply at position s_pos.

        Args:
          s_pos: Position on border in curve_parameter ds.

        Returns:
          An array with coefficients [a, b, c, d] for the polynomial w = a + b*ds + c*ds² + d*ds³

        """
        width_idx = self._get_width_index(s_pos, is_last_pos)
        return self.width_coefficients[width_idx]

    # NOTE: might by more efficient to calculate each border once
    # instead of recalculating them over and over.
    @lru_cache(maxsize=200000)
    def calc(self, s_pos: float, width_offset: float = 0.0, is_last_pos: bool = False):
        """Calculate the Cartesian coordinates and the tangential direction of
        the border by calculating position of reference border at s_pos
        and then adding the width in orthogonal direction to the reference position.

        Args:
          s_pos: Position s_pos (specified in curve parameter ds)
            where to calculate the cartesian coordinates on the border.
          width_offset: Offset to add to calculated width at position s_pos.

        Returns:
          (x,y) tuple of cartesian coordinates and the direction angle in radians.
        """
        # Last reference has to be a reference geometry (PlanView)
        # Offset of all inner lanes (Border)
        # calculate position of reference border
        if np.isclose(s_pos, 0):
            s_pos = 0

        try:
            ref_coord, tang_angle = self.reference.calc(
                self.ref_offset + s_pos, is_last_pos=is_last_pos
            )
        except TypeError:
            ref_coord, tang_angle = self.reference.calc(self.ref_offset + s_pos)

        if not self.width_coefficients or not self.width_coefficient_offsets:
            raise Exception("No entries for width definitions.")

        # Find correct coefficients
        # find which width segment is at s_pos
        width_idx = self._get_width_index(s_pos, is_last_pos)

        # Calculate width at s_pos
        distance = (
            np.polynomial.polynomial.polyval(
                s_pos - self.width_coefficient_offsets[width_idx],
                self.width_coefficients[width_idx],
            )
            + width_offset
        )

        # New point is in orthogonal direction
        ortho = tang_angle + np.pi / 2
        coord = ref_coord + np.array(
            [distance * np.cos(ortho), distance * np.sin(ortho)]
        )

        return coord, tang_angle
