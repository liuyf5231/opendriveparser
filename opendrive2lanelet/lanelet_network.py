# -*- coding: utf-8 -*-

"""Module to enhance LaneletNetwork class
so it can be used for conversion from the opendrive format."""


from typing import Tuple, List
from queue import Queue
import numpy as np

from commonroad.scenario.lanelet import LaneletNetwork
from opendrive2lanelet.lanelet import ConversionLanelet

__author__ = "Benjamin Orthen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


class ConversionLaneletNetwork(LaneletNetwork):
    """Add functions to LaneletNetwork which
    further enable it to modify its Lanelets."""

    def remove_lanelet(self, lanelet_id: str, remove_references: bool = False):
        """Remove a lanelets with the specific lanelet_id
        from the _lanelets dict.

        Args:
          lanelet_id: id of lanelet to be removed.
          remove_references: Also remove references which point to to be removed lanelet.

        Returns:
          None

        """
        del self._lanelets[lanelet_id]
        if remove_references:
            for lanelet in self.lanelets:
                lanelet.predecessor[:] = [
                    pred for pred in lanelet.predecessor if pred != lanelet_id
                ]

                lanelet.successor[:] = [
                    succ for succ in lanelet.successor if succ != lanelet_id
                ]
                if lanelet.adj_right == lanelet_id:
                    lanelet.adj_right = None
                if lanelet.adj_left == lanelet_id:
                    lanelet.adj_left = None

    def find_lanelet_by_id(self, lanelet_id) -> ConversionLanelet:
        """Find a lanelet for a given lanelet_id.
        Disable natural number check of parent class.

        Args:
          lanelet_id: The id of the lanelet to find

        Returns:
          The lanelet object if the id exists and None otherwise

        """
        return self._lanelets.get(lanelet_id)

    def prune_network(self):
        """Remove references in predecessor, successor etc. to
        non existing lanelets.

        Args:

        Returns:

        """
        self._delete_zero_width_parametric_lanes()

        lanelet_ids = [x.lanelet_id for x in self.lanelets]

        for lanelet in self.lanelets:
            lanelet.predecessor[:] = [
                pred for pred in lanelet.predecessor if pred in lanelet_ids
            ]
            lanelet.successor[:] = [
                succ for succ in lanelet.successor if succ in lanelet_ids
            ]
            if lanelet.adj_left not in lanelet_ids:
                lanelet.adj_left = None
            if lanelet.adj_right not in lanelet_ids:
                lanelet.adj_right = None

    def _delete_zero_width_parametric_lanes(self):
        """Remove all ParametricLaneGroup which have zero width at every point from
        this network.
        """
        for lanelet in self.lanelets:
            if lanelet.has_zero_width_everywhere():
                if lanelet.adj_right:
                    adj_right = self.find_lanelet_by_id(lanelet.adj_right)
                    if adj_right:
                        adj_right.adj_left = lanelet.adj_left

                if lanelet.adj_left:
                    adj_left = self.find_lanelet_by_id(lanelet.adj_left)
                    if adj_left:
                        adj_left.adj_right = lanelet.adj_right

                self.remove_lanelet(lanelet.lanelet_id, remove_references=True)

    def update_lanelet_id_references(self, old_id: str, new_id: str):
        """Update all references to the old lanelet_id with the new_lanelet_id.

        Args:
          old_id: Old lanelet_id which has changed.
          new_id: New lanelet_id the old_id has changed into.

        """

        for lanelet in self.lanelets:
            lanelet.predecessor[:] = [
                new_id if pred == old_id else pred for pred in lanelet.predecessor
            ]

            lanelet.successor[:] = [
                new_id if succ == old_id else succ for succ in lanelet.successor
            ]

            if lanelet.adj_right == old_id:
                lanelet.adj_right = new_id

            if lanelet.adj_left == old_id:
                lanelet.adj_left = new_id

    def concatenate_possible_lanelets(self):
        """Iterate trough lanelets in network and concatenate possible lanelets together.

        Check for each lanelet if it can be concatenated with its successor and
        if its neighbors can be concatenated as well. If yes, do the concatenation.

        """
        concatenate_lanelets = []
        for lanelet in self.lanelets:

            if lanelet.adj_right is None:
                possible_concat_lanes = self.check_concatenation_potential(
                    lanelet, "left"
                )
            elif lanelet.adj_left is None:
                possible_concat_lanes = self.check_concatenation_potential(
                    lanelet, "right"
                )
            if possible_concat_lanes:
                concatenate_lanelets.append(possible_concat_lanes)

        # this dict saves the lanelet_ids which point to another lanelet_id
        # because they were concatenated together and the resulting lanelet
        # can only have one lanelet_id.
        # key in dict has been renamed to value
        replacement_ids = dict()

        for possible_concat_lanes in concatenate_lanelets:
            # prevent chains of more than one lanelet being renamed
            replacement_ids = {
                k: replacement_ids.get(v, v) for k, v in replacement_ids.items()
            }

            possible_concat_lanes = [
                (
                    replacement_ids.get(pair[0], pair[0]),
                    replacement_ids.get(pair[1], pair[1]),
                )
                for pair in possible_concat_lanes
            ]
            replacement_ids.update(
                self._concatenate_lanelet_pairs_group(possible_concat_lanes)
            )

    def _concatenate_lanelet_pairs_group(self, lanelet_pairs: list) -> dict:
        """Concatenate a group of lanelet_pairs, with setting correctly the new lanelet_ids
        at neighbors.

        Args:
          lanelet_pairs: List with tuples of lanelet_ids which should be concatenated.

        Returns:
          Dict with information which lanelet_id was converted to a new one.

        """

        new_lanelet_ids = dict()
        for pair in lanelet_pairs:
            if pair[0] == pair[1]:
                continue
            lanelet_1 = self.find_lanelet_by_id(pair[0])
            lanelet_2 = self.find_lanelet_by_id(pair[1])
            lanelet_1.concatenate(lanelet_2)

            self.remove_lanelet(pair[1])

            # each reference to lanelet_2 should point to lanelet_id
            # of new_lanelet instead
            self.update_lanelet_id_references(
                lanelet_2.lanelet_id, lanelet_1.lanelet_id
            )
            # update dict to show which lanelet_id changed to which
            new_lanelet_ids[pair[1]] = pair[0]

        return new_lanelet_ids

    def join_and_split_possible_lanes(self):
        """Move lanelet boundaries for lanelet splits or joins.

        This method provides the functionality to modify the lane boundaries if
        a lane merges into another lane or splits from another lane.
        """
        # Condition for lane merge:
        # left and right vertices at end or beginning are the same
        js_targets = []
        for lanelet in self.lanelets:

            lanelet_split, lanelet_join = False, False
            if not lanelet.predecessor and np.allclose(
                lanelet.left_vertices[0], lanelet.right_vertices[0]
            ):
                lanelet_split = True

            if not lanelet.successor and np.allclose(
                lanelet.left_vertices[-1], lanelet.right_vertices[-1]
            ):
                lanelet_join = True

            if lanelet_join or lanelet_split:
                js_targets.append(
                    _JoinSplitTarget(self, lanelet, lanelet_split, lanelet_join)
                )

        for js_target in js_targets:
            js_target.determine_apt_js_pairs()
            js_target.move_borders()
            js_target.add_adjacent_predecessor_or_successor()

    def predecessor_is_neighbor_of_neighbors_predecessor(
        self, lanelet: "ConversionLanelet"
    ) -> bool:
        """Checks if neighbors of predecessor are the successor of the adjacent neighbors
        of the lanelet.

        Args:
          lanelet: Lanelet to check neighbor requirement for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        if not self.has_unique_pred_succ_relation(-1, lanelet):
            return False

        predecessor = self.find_lanelet_by_id(lanelet.predecessor[0])
        return self.successor_is_neighbor_of_neighbors_successor(predecessor)

    def add_successors_to_lanelet(
        self, lanelet: ConversionLanelet, successor_ids: List[str]
    ):
        """Add a successor to a lanelet, but add the lanelet also to the predecessor
        of the succesor.

        Args:
          lanelet: Lanelet to add successor to.
          successor_ids: Id of successor to add to lanelet.
        """
        for successor_id in successor_ids:
            lanelet.successor.append(successor_id)
            successor = self.find_lanelet_by_id(successor_id)
            successor.predecessor.append(lanelet.lanelet_id)

    def add_predecessors_to_lanelet(
        self, lanelet: ConversionLanelet, predecessor_ids: List[str]
    ):
        """Add a successor to a lanelet, but add the lanelet also to the predecessor
        of the succesor.

        Args:
          lanelet: Lanelet to add successor to.
          predecessor_id: Id of successor to add to lanelet.
        """
        for predecessor_id in predecessor_ids:
            lanelet.predecessor.append(predecessor_id)
            predecessor = self.find_lanelet_by_id(predecessor_id)
            predecessor.successor.append(lanelet.lanelet_id)

    def check_concatenation_potential(
        self, lanelet: ConversionLanelet, adjacent_direction: str
    ) -> list:
        """Check if lanelet could be concatenated with its successor.

        Args:
          lanelet: Lanelet to check concatenation potential with its successor
          adjacent_direction: "Left" or "Right", determinating which lanelet

        Returns:
          A list of pairs of lanelets which can be concatenated. None if it is not possible.

        """
        mergeable_lanelets = []
        neighbor_ok = self.successor_is_neighbor_of_neighbors_successor(lanelet)
        if not neighbor_ok:
            return None

        if adjacent_direction == "left":
            mergeable_lanelets.append((lanelet.lanelet_id, lanelet.successor[0]))
            if lanelet.adj_left is None:
                return mergeable_lanelets
            if lanelet.adj_left_same_direction:
                next_neighbor = self.find_lanelet_by_id(lanelet.adj_left)
                new_direction = "left"
            else:
                next_neighbor = self.find_lanelet_by_id(
                    self.find_lanelet_by_id(lanelet.adj_left).predecessor[0]
                )
                new_direction = "right"

        else:
            mergeable_lanelets.append((lanelet.lanelet_id, lanelet.successor[0]))
            if lanelet.adj_right is None:
                return mergeable_lanelets
            if lanelet.adj_right_same_direction:
                next_neighbor = self.find_lanelet_by_id(lanelet.adj_right)
                new_direction = "right"
            else:
                next_neighbor = self.find_lanelet_by_id(
                    self.find_lanelet_by_id(lanelet.adj_right).predecessor[0]
                )
                new_direction = "left"

        recursive_merge = self.check_concatenation_potential(
            next_neighbor, new_direction
        )
        if recursive_merge is None:
            return None

        mergeable_lanelets.extend(recursive_merge)
        return mergeable_lanelets

    def successor_is_neighbor_of_neighbors_successor(
        self, lanelet: ConversionLanelet
    ) -> bool:
        """Checks if neighbors of successor are the successor of the adjacent neighbors
        of the lanelet.

        Args:
          lanelet: Lanelet to check specified relation for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        if not self.has_unique_pred_succ_relation(1, lanelet):
            return False

        return self.adj_left_consistent_nb(lanelet) and self.adj_right_consistent_nb(
            lanelet
        )

    def has_unique_pred_succ_relation(
        self, direction: int, lanelet: ConversionLanelet
    ) -> bool:
        """Checks if lanelet has only one successor/predecessor and the
        successor/predecessor has only one predecessor/successor, s.t.
        it is a one-to-one relation.

        Args:
          direction: 1 if the successor should be checked.
        -1 (or all other values) for the predecessor.
          lanelet_network: Network to search for lanelets by ids.
          direction: int:
          lanelet_network: "LaneletNetwork":

        Returns:
          True if the relation is unique, False otherwise.

        """
        if direction == 1:
            neighbors = lanelet.successor
        else:
            neighbors = lanelet.predecessor

        # check if neighbor is only one
        if neighbors is None or len(neighbors) != 1:
            return False

        # get lanelet object with id
        neighbor = self.find_lanelet_by_id(neighbors[0])

        # get the neighbor of the neighbor
        nb_neighbor = neighbor.predecessor if direction == 1 else neighbor.successor

        # check if nb_neighbor has one neighbor in proper direction
        if nb_neighbor is None or len(nb_neighbor) != 1:
            return False

        # return True if these ids are the same (they should be!)
        return lanelet.lanelet_id == nb_neighbor[0]

    def adj_right_consistent_nb(self, lanelet: ConversionLanelet) -> bool:
        """Checks if right neighbor of successor is the successor
        of the right adjacent neighbor of the lanelet.

        Args:
          lanelet: Lanelet to check specified relation for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        successor = self.find_lanelet_by_id(lanelet.successor[0])
        adj_right = self.find_lanelet_by_id(lanelet.adj_right)
        if adj_right:
            if lanelet.adj_right_same_direction:
                if not self.has_unique_pred_succ_relation(1, adj_right):
                    return False
                if adj_right.successor[0] != successor.adj_right:
                    return False
            else:
                if not lanelet.has_unique_pred_succ_relation(-1, adj_right):
                    return False
                if adj_right.predecessor[0] != successor.adj_right:
                    return False
        else:
            return successor.adj_right is None
        return True

    def adj_left_consistent_nb(self, lanelet: ConversionLanelet) -> bool:
        """Checks if left neighbor of successor is the successor of the
        left adjacent neighbor of the lanelet.

        Args:
          lanelet: Lanelet to check specified relation for.

        Returns:
          True if this neighbor requirement is fulfilled.

        """
        successor = self.find_lanelet_by_id(lanelet.successor[0])
        adj_left = self.find_lanelet_by_id(lanelet.adj_left)
        if adj_left:
            if lanelet.adj_left_same_direction:
                if not self.has_unique_pred_succ_relation(1, adj_left):
                    return False
                if adj_left.successor[0] != successor.adj_left:
                    return False
            else:
                if not self.has_unique_pred_succ_relation(-1, adj_left):
                    return False
                if adj_left.predecessor[0] != successor.adj_left:
                    return False
        else:
            return successor.adj_left is None
        return True


class _JoinSplitTarget:
    """Class to integrate joining/splitting of lanelet borders.

    Provides methods to determine the lanelets with which the
    join and/or split can be performed. Additionally a method to
    change the borders of the determined lanelets.

    Attributes:
      main_lanelet (ConversionLanelet): Lanelet where split starts or join ends.
      lanelet_network (ConversionLaneletNetwork): LaneletNetwork where join/split occurs.
      _mode (int): Number denoting if join (0), split (1), or join and split (2) occurs.
      change_width (float): Width at start of split or end of join.
        Is list with two elements, [split_width, join_width] if _mode == 2
      linking_side (str): Side on which the split/join happens (either "left" or "right")
      _js_pairs (list): List of :class:`._JoinSplitPair` elements.
      _single_lanelet_operation (bool): Indicates whether only one lanelet and
        its adjacent lanelet can be used for the join/split.
    """

    def __init__(
        self,
        lanelet_network: ConversionLaneletNetwork,
        main_lanelet: ConversionLanelet,
        split: bool,
        join: bool,
    ):
        self.main_lanelet = main_lanelet
        self.lanelet_network = lanelet_network
        if split and join:
            self._mode = 2
        elif split:
            self._mode = 1
        else:
            self._mode = 0
        self.change_width = None
        self.linking_side = None
        self._js_pairs = []
        self._single_lanelet_operation = False

    @property
    def split(self):
        """Lanelet splits at start.

        Returns:
          True if lanelet splits from other lanelet at start.
        """
        return self._mode >= 1

    @property
    def join(self):
        """Lanelet joins at end.

        Returns:
          True if lanelet joins to other lanelet at end.
        """
        return self._mode != 1

    @property
    def split_and_join(self) -> bool:
        """Lanelet splits at start and joins at end.

        Returns:
          True if it has a join and a split.
        """
        return self.split and self.join

    def use_only_single_lanelet(self) -> bool:
        """Only single lanelet can be used for join/split.

        Returns:
          True if only one can be used.
        """
        return self._single_lanelet_operation and self.split_and_join

    def _find_lanelet_by_id(self, lanelet_id: str) -> ConversionLanelet:
        """Run :func:`.ConversionLaneletNetwork.find_lanelet_by_id` of self.lanelet_network.

        Returns:
          Lanelet matching the lanelet_id.
        """
        return self.lanelet_network.find_lanelet_by_id(lanelet_id)

    def complete_js_interval_length(self) -> float:
        """Calculate length of interval where join/split changes the border.

        Returns:
          Length of interval.
        """
        length = 0
        for js_pair in self._js_pairs:
            length += js_pair.change_interval[1] - js_pair.change_interval[0]

        return length

    def adjacent_width(self, is_split: bool) -> float:
        """Get width of adjacent lanelet at start of split or end of join.

        Returns:
          Width of adjacent lanelet at start or end.
        """
        if is_split:
            return self._js_pairs[0].adjacent_lanelet.calc_width_at_start()
        return self._js_pairs[0].adjacent_lanelet.calc_width_at_end()

    def add_adjacent_predecessor_or_successor(self):
        """Add the predecessor or successor of the adjacent lanelet to the main lanelet.

        This reflects that after the split, the main lanelet is also a successor
        of the predecessor of its adjacent lanelet.
        For a join, the main lanelet is a predecessor of the successor of its
        adjacent lanelet.
        """

        if not self._js_pairs:
            return
        lanelet = self._js_pairs[0].lanelet
        adjacent_lanelet = self._js_pairs[0].adjacent_lanelet
        if self.split:
            self.lanelet_network.add_predecessors_to_lanelet(
                lanelet, adjacent_lanelet.predecessor
            )

        if self.join:
            self.lanelet_network.add_successors_to_lanelet(
                lanelet, adjacent_lanelet.successor
            )

    def move_borders(self):
        """Move borders of lanelets to reflect the split/join.

        All lanelet pairs in self._js_pairs are used for the border movement.
        """

        if not self._js_pairs:
            return

        if self.split_and_join:
            self._move_borders_if_split_and_join()
        else:
            self._move_borders_if_split_or_join()

    def _move_borders_if_split_or_join(self):
        """Move borders of lanelets if it is not split and join.

        Interpolate width interval for each js_pair.
        Then move the borders of its lanelet.
        """
        length = self.complete_js_interval_length()
        adj_width = self.adjacent_width(is_split=self.split)
        if self.join:
            js_pairs = list(reversed(self._js_pairs))
            # norm running position so that running_pos + pos_start
            # is at zero at first js_pair
            running_pos = -1 * self._js_pairs[0].change_interval[0]
            width_start = self.change_width
            width_end = adj_width
        else:
            js_pairs = self._js_pairs
            running_pos = 0
            width_start = adj_width
            width_end = self.change_width
        for js_pair in js_pairs:
            [pos_start, pos_end] = js_pair.change_interval
            distance = np.interp(
                [pos_start + running_pos, pos_end + running_pos],
                [0, length],
                [width_start, width_end],
            )
            js_pair.move_border(width=distance, linking_side=self.linking_side)
            running_pos += pos_end - pos_start

    def _move_borders_if_split_and_join(self):
        """Move borders of lanelets if it is split and join.

        Calculate the new vertices twice:
        1. Only for the split (first pair in self._js_pairs)
        2. Only for the join (second pair in self._js_pairs)
        Then talk the first half of the vertices of the split and
        the seconds half of the vertices of the join and merge them.
        """
        lanelet = self._js_pairs[0].lanelet

        start_width_split = self.adjacent_width(is_split=True)
        lanelet_split = self._js_pairs[0].move_border(
            width=[start_width_split, self.change_width[0]],
            linking_side=self.linking_side,
        )
        left_vertices = lanelet_split.left_vertices
        right_vertices = lanelet_split.right_vertices
        center_vertices = lanelet_split.center_vertices
        start_width_join = self.adjacent_width(is_split=False)
        self._js_pairs[1].move_border(
            width=[self.change_width[1], start_width_join],
            linking_side=self.linking_side,
        )

        # take first half of lanelet which does the split
        # take second half of lanelet which does the join
        half_length = int(left_vertices[:, 0].size / 2)
        lanelet.left_vertices = np.vstack(
            (left_vertices[:half_length, :], lanelet.left_vertices[half_length:, :])
        )
        lanelet.right_vertices = np.vstack(
            (right_vertices[:half_length, :], lanelet.right_vertices[half_length:, :])
        )
        lanelet.center_vertices = np.vstack(
            (center_vertices[:half_length, :], lanelet.center_vertices[half_length:, :])
        )

    def determine_apt_js_pairs(self):
        """Determine pairs of lanelet and adjacent lanelet for the join/split.

        Add lanelets as long as one of the break conditions is not matched.
        The determined pairs are saved in self._js_pairs.
        """
        # for first lanelet
        adjacent_lanelet = self._determine_main_adjacent_lanelet()
        if not adjacent_lanelet:
            return
        lanelet = self.main_lanelet
        while True:
            algo_has_finished = self._add_join_split_pair(lanelet, adjacent_lanelet)
            if algo_has_finished or self.use_only_single_lanelet():
                break
            if (
                self.split
                and self.lanelet_network.successor_is_neighbor_of_neighbors_successor(
                    lanelet
                )
                # and lanelet.successor[0] not in global_adjacent_lanelets
            ):
                lanelet = self._find_lanelet_by_id(lanelet.successor[0])
                adjacent_lanelet = self._find_lanelet_by_id(
                    adjacent_lanelet.successor[0]
                )
            elif self.join and (
                self.lanelet_network.predecessor_is_neighbor_of_neighbors_predecessor(
                    lanelet
                )
                # and lanelet.predecessor[0] not in global_adjacent_lanelets
            ):
                lanelet = self._find_lanelet_by_id(lanelet.predecessor[0])
                adjacent_lanelet = self._find_lanelet_by_id(
                    adjacent_lanelet.predecessor[0]
                )

            else:
                break

    def _add_join_split_pair(
        self, lanelet: ConversionLanelet, adjacent_lanelet: ConversionLanelet
    ) -> bool:
        """Add a pair of lanelet and adjacent lanelet to self._js_pairs.

        Decide if it is advisable to add another pair to increase join/split area.

        Args:
          lanelet: Lanelet to be added.
          adjacent_lanelet: Lanelet adjacent to lanelet to be added.
        Returns:
          Indicator whether this was the last pair to be added. False means
           it is advisable to add another lanelet pair.
        """
        if self.split_and_join:
            # one for split at start of lanelet
            change_pos, change_width = lanelet.optimal_join_split_values(
                is_split=True,
                split_and_join=self.split_and_join,
                reference_width=adjacent_lanelet.calc_width_at_start(),
            )
            self._js_pairs.append(
                _JoinSplitPair(lanelet, adjacent_lanelet, [0, change_pos])
            )
            self.change_width = [change_width]
            # one for join at the end of the lanelet
            change_pos, change_width = lanelet.optimal_join_split_values(
                is_split=False,
                split_and_join=self.split_and_join,
                reference_width=adjacent_lanelet.calc_width_at_end(),
            )
            self._js_pairs.append(
                _JoinSplitPair(lanelet, adjacent_lanelet, [change_pos, lanelet.length])
            )
            self.change_width.append(change_width)
            return True

        adjacent_width = (
            adjacent_lanelet.calc_width_at_start()
            if self.split
            else adjacent_lanelet.calc_width_at_end()
        )
        change_pos, change_width = lanelet.optimal_join_split_values(
            is_split=self.split,
            split_and_join=self.split_and_join,
            reference_width=adjacent_width,
        )
        if self.change_width is not None and change_width < self.change_width:
            # algorithm to add lanelet should terminate
            return True
        self.change_width = change_width
        if self.split:
            self._js_pairs.append(
                _JoinSplitPair(lanelet, adjacent_lanelet, [0, change_pos])
            )
            if np.isclose(lanelet.length, change_pos):
                return False
        else:
            self._js_pairs.append(
                _JoinSplitPair(lanelet, adjacent_lanelet, [change_pos, lanelet.length])
            )
            if np.isclose(0, change_pos):
                return False
        return True

    def _determine_main_adjacent_lanelet(self) -> ConversionLanelet:
        """Determine which is the adjacent lanelet to the main lanelet.

        Returns:
          The corresponding adjacent lanelet.
        """
        lanelet = self.main_lanelet
        potential_adjacent_lanelets = Queue()
        checked_lanelets = 0
        if lanelet.adj_left is not None and lanelet.adj_left_same_direction:
            potential_adjacent_lanelets.put(
                {"lanelet_id": lanelet.adj_left, "linking_side": "right"}
            )
            checked_lanelets -= 1
        if lanelet.adj_right is not None and lanelet.adj_right_same_direction:
            potential_adjacent_lanelets.put(
                {"lanelet_id": lanelet.adj_right, "linking_side": "left"}
            )
            checked_lanelets -= 1

        while potential_adjacent_lanelets.qsize() > 0:
            adjacent_lanelet = self._check_next_adjacent_lanelet(
                potential_adjacent_lanelets
            )
            checked_lanelets += 1

            if checked_lanelets > 0:
                # adjacent lanelet is not next neighbor
                # successor of adjacent lanelet cant be used
                self._single_lanelet_operation = True
            if adjacent_lanelet is not None:
                # found appropriate adjacent lanelet
                return adjacent_lanelet

        return None

    def _check_next_adjacent_lanelet(
        self, potential_adjacent_lanelets: Queue
    ) -> ConversionLanelet:
        """Check next lanelet if it can act as adjacent lanelet to the main lanelet.

        If not, add its left and right neighbor, if they exists, to the potential_adjacent_lanelets Queue.

        Args:
          potential_adjacent_lanelets: Queue with dicts containing the pontential lanelets.
        Returns:
          Lanelet which fulfills the conditions if it exists, else None
        """
        adj_target = potential_adjacent_lanelets.get()
        adj_lanelet = self._find_lanelet_by_id(adj_target.get("lanelet_id"))
        linking_side = adj_target.get("linking_side")
        return_flag = 0

        if not adj_lanelet:
            return None
        if self.join:
            adj_width = adj_lanelet.calc_width_at_end()

        if self.split:
            adj_width = adj_lanelet.calc_width_at_start()
        if adj_width > 0:
            self.linking_side = adj_target.get("linking_side")
            return_flag = 1

        if return_flag:
            return adj_lanelet

        next_adj_neighbor = (
            adj_lanelet.adj_left if linking_side == "right" else adj_lanelet.adj_right
        )
        if next_adj_neighbor:
            potential_adjacent_lanelets.put(
                {"lanelet_id": next_adj_neighbor, "linking_side": linking_side}
            )
        return None


class _JoinSplitPair:
    "Pair of lanelet whose border is changed and its adjacent neighbor."

    def __init__(self, lanelet, adjacent_lanelet, change_interval):
        self.lanelet = lanelet
        self.adjacent_lanelet = adjacent_lanelet
        self.change_interval = change_interval

    def move_border(
        self, width: Tuple[float, float], linking_side: str
    ) -> ConversionLanelet:
        """Move border of self.lanelet.

        Args:
          width: Start and end value of new width of lanelet.
          linking_side: Side on which the split/join happens (either "left" or "right").
        Returns:
          Resulting lanelet after border movement.
        """
        self.lanelet.move_border(
            mirror_border=linking_side,
            mirror_interval=self.change_interval,
            distance=width,
            adjacent_lanelet=self.adjacent_lanelet,
        )
        return self.lanelet
