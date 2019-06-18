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


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-w',
        '--whitelist',
        nargs='*',
        help='List of whitelist interfaces separated by spaces',
        required=True,
    )
    parser.add_argument(
        '-f',
        '--file',
        help='Path to the file to store the XML',
        required=True,
    )
    args = parser.parse_args()
    interface_whitelist = args.whitelist
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
            item_func=(
                lambda parent_name: 'address'
                if parent_name == 'interfaceWhiteList'
                else parent_name
            ),
            custom_root='profiles',
            attr_type=False,
        ).decode()
    )

    part_el = xml.documentElement.getElementsByTagName('participant').item(0)
    part_el.setAttribute('profile_name', 'apex_profile')
    part_el.setAttribute('is_default_profile', 'true')
    with open(xml_file, 'w') as f:
        f.write(xml.toprettyxml(indent=' '*2).split('\n', 1)[1])
