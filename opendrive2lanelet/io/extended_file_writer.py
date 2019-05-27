# -*- coding: utf-8 -*-
"""
Extend the file_writer module from commonroad-io to
include methods to write directly to a file.

Increase precision of floats written to file by setting ctx.prec = 10.

Avoid RAM problem with minidom reparsing huge xml strings.
"""

import io
import os
import xml.etree.ElementTree as et
from lxml import etree, objectify

from commonroad.common.file_writer import CommonRoadFileWriter, ctx

# TODO: use lxml in commonroad file_writer

__author__ = "Benjamin Orthen"
__copyright__ = "TUM Cyber-Physical Systems Group"
__credits__ = ["Priority Program SPP 1835 Cooperative Interacting Automobiles"]
__version__ = "1.0.2"
__maintainer__ = "Benjamin Orthen"
__email__ = "commonroad-i06@in.tum.de"
__status__ = "Released"


ctx.prec = 10


class ExtendedCommonRoadFileWriter(CommonRoadFileWriter):
    """Extends CommonRoadFileWriter to write to file like objects."""

    # TODO: integrate this in file_writer.py of commonroad?
    def __init__(
        self,
        scenario,
        planning_problem_set=None,
        author="",
        affiliation="",
        source="",
        tags="",
    ):
        super().__init__(
            scenario, planning_problem_set, author, affiliation, source, tags
        )

    def _dump(self):
        """Dump the root_node of the xml representation to str.
        Overwrites the parent method to not use minidom.
        This is due to the fact minidom uses too much memory when
        processing big xml files.

        Args:

        Returns:
          str: Serialized xml data.

        """
        # output_string = etree.tostring(
        # self._root_node, encoding="utf-8", pretty_print=True
        # )
        output_string = et.tostring(self._root_node, encoding="unicode")
        # TODO: remove this method as soon as commonroad publishes new version
        return output_string

    def write_scenario_to_file_io(self, file_io: io.IOBase):
        """Write a scenario without planning-problem to file_io.

        Args:
          file_io: File to write to.

        """
        self._write_header()
        self._add_all_objects_from_scenario()
        self._write_xml_output_to_file(file_io)

    def write_to_file_io(self, file_io: io.IOBase):
        """Write a scenario including planning-problem to file_io.

        Args:
          file_io: File to write to.

        """
        self._write_header()
        self._add_all_objects_from_scenario()
        self._add_all_planning_problems_from_planning_problem_set()
        self._write_xml_output_to_file(file_io)

    def _write_xml_output_to_file(self, file_io: io.IOBase):
        """Write the dump from self._dump() to file_io.

        Args:
          file_io: File to write to.

        """
        output_str = self._dump()
        ExtendedCommonRoadFileWriter.check_validity_of_commonroad_file(output_str)
        file_io.write(output_str)

    @staticmethod
    def check_validity_of_commonroad_file(commonroad_str: str):
        """Check the validity of a generated xml_string in terms of
        commonroad with an existing XSD schema.
        Throw an error if it is not valid.

        Args:
          commonroad_str: XML formatted string which should be checked.

        """
        with open(
            os.path.dirname(os.path.abspath(__file__)) + "/commonroad_validity.xsd",
            "rb",
        ) as schema_file:
            schema = etree.XMLSchema(etree.parse(schema_file))

        parser = objectify.makeparser(schema=schema, encoding="utf-8")

        try:
            etree.fromstring(commonroad_str, parser)
        except etree.XMLSyntaxError as error:
            raise Exception(
                "Could not produce valid CommonRoad file! Error: {}".format(error.msg)
            )
