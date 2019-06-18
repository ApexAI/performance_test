#!/usr/bin/env python

# Copyright 2017 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Generate DEFAULT XML file for apex tests."""
import argparse
from xml.dom.minidom import parseString

from dicttoxml import dicttoxml


def item_names(parent_name):
    """
    Get the tag of an item based on its parent.

    :param parent_name: The name of the parent.
    :return: The item tag.
    """
    if parent_name == 'interfaceWhiteList':
        return 'address'
    elif parent_name == 'transport_descriptors':
        return 'transport_descriptors'


if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-w',
        '--whitelist',
        help='List of whitelist interfaces separated by comma with no spaces',
        required=True,
    )
    parser.add_argument(
        '-f',
        '--file',
        help='Path to the file to store the XML',
        required=True,
    )
    args = parser.parse_args()
    interface_whitelist = args.whitelist.split(',')
    xml_file = args.file

    profiles = {
        'transport_descriptors': {
                'transport_descriptor': {
                    'transport_id': 'apex_transport_descriptor',
                    'type': 'UDPv4',
                    'interfaceWhiteList': interface_whitelist,
                },
        },
        'participant': {
            'rtps': {
                'name': 'Apex participant profile',
                'useBuiltinTransports': 'false',
                'userTransports': {
                    'transport_id': 'apex_transport_descriptor',
                },
            },
        },
    }

    xml = parseString(
        dicttoxml(
            profiles,
            item_func=item_names,
            custom_root='profiles',
            attr_type=False,
        ).decode()
    ).toprettyxml(indent='    ')
    parsed_xml = xml.split('\n', 1)[1]
    lines = parsed_xml.split('\n')

    with open(xml_file, 'w') as f:
        for line in lines:
            if line == '':
                continue
            line = line.replace(
                '<participant',
                (
                    '<participant profile_name="apex_profile"' +
                    ' is_default_profile="true"'
                )
            )
            f.write('{}\n'.format(line))
