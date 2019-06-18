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

"""
Generate DEFAULT XML file for apex tests.

Helper script to generate a DEFAULT_FASTRTPS_PROFILES.xml XML file to add an
interface whitelist to be used in the Fast-RTPS communication plugin. The
script takes two optional arguments: a list of IP addresses (to create the
interface whitelist), and a directory to place the resulting
DEFAULT_FASTRTPS_PROFILES.xml (this directory defaults to the current
directory).

:Usage:
    generate_apex_xml.py [-h] [-w WHITELIST [WHITELIST ...]] [-d DIRECTORY]

    optional arguments:
    -h, --help            show this help message and exit
    -w WHITELIST [WHITELIST ...], --whitelist WHITELIST [WHITELIST ...]
                            List of whitelist interfaces separated by spaces
                            (default: None)
    -d DIRECTORY, --directory DIRECTORY
                            Directory to store the XML file (default: .)

:Example:
    generate_apex_xml.py -w 192.168.1.1 127.0.0.1 -f some_directory

    The previous command would place a DEFAULT_FASTRTPS_PROFILES.xml file in
    the 'some_directory' directory. The resulting output would be:
        <profiles>
        <transport_descriptors>
            <transport_descriptor>
            <transport_id>apex_transport_descriptor</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>192.168.1.11</address>
                <address>127.0.0.1</address>
            </interfaceWhiteList>
            </transport_descriptor>
        </transport_descriptors>
        <participant is_default_profile="true" profile_name="apex_profile">
            <rtps>
            <name>Apex participant profile</name>
            <useBuiltinTransports>false</useBuiltinTransports>
            <userTransports>
                <transport_id>apex_transport_descriptor</transport_id>
            </userTransports>
            </rtps>
        </participant>
        </profiles>
"""
import argparse
import os
from xml.dom.minidom import Document


def directory_type(directory):
    """
    Check whether the argument is a directory.

    :param directory: The directory path.
    :return: The directory path without ending /.
    :raise: argparse.ArgumentTypeError if the argument is not a directory.
    """
    if directory.endswith('/'):
        directory = directory[:-1]
    if not os.path.isdir(directory):
        raise argparse.ArgumentTypeError(
            'Cannot find directory "{}"'.format(directory)
        )
    return directory


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument(
        '-w',
        '--whitelist',
        nargs='+',
        help='List of whitelist interfaces separated by spaces',
        required=False,
    )
    parser.add_argument(
        '-d',
        '--directory',
        type=directory_type,
        help='Directory to store the XML file',
        required=False,
        default='.'
    )
    args = parser.parse_args()
    interface_whitelist = args.whitelist
    xml_file = '{}/DEFAULT_FASTRTPS_PROFILES.xml'.format(args.directory)

    doc = Document()
    # Top level tag profiles
    profiles = doc.createElement('profiles')

    # Transport desciptors
    transport_descriptors = doc.createElement('transport_descriptors')

    # Transport desciptor
    transport_descriptor = doc.createElement('transport_descriptor')
    # Transport desciptor ID
    transport_id = doc.createElement('transport_id')
    transport_id.appendChild(doc.createTextNode('apex_transport_descriptor'))
    transport_descriptor.appendChild(transport_id)
    # Transport desciptor type
    transport_type = doc.createElement('type')
    transport_type.appendChild(doc.createTextNode('UDPv4'))
    transport_descriptor.appendChild(transport_type)
    # Transport desciptor whitelist
    if interface_whitelist:
        whitelists = doc.createElement('interfaceWhiteList')
        for element in interface_whitelist:
            address = doc.createElement('address')
            address.appendChild(doc.createTextNode(element))
            whitelists.appendChild(address)
        transport_descriptor.appendChild(whitelists)
    # Append transport descriptor to transport_descriptor
    transport_descriptors.appendChild(transport_descriptor)
    # Append transport descriptors to profile
    profiles.appendChild(transport_descriptors)

    # Participant
    participant = doc.createElement('participant')
    # Participant attributes
    participant.setAttribute('profile_name', 'apex_profile')
    participant.setAttribute('is_default_profile', 'true')
    # Participant RTPS
    rtps = doc.createElement('rtps')
    # Participant RTPS name
    name = doc.createElement('name')
    name.appendChild(doc.createTextNode('Apex participant profile'))
    rtps.appendChild(name)
    # Participant RTPS useBuiltinTransports flag
    use_btp = doc.createElement('useBuiltinTransports')
    use_btp.appendChild(doc.createTextNode('false'))
    rtps.appendChild(use_btp)
    # Participant RTPS user transports
    user_transports = doc.createElement('userTransports')
    # Participant RTPS user transports ID
    transport_id = doc.createElement('transport_id')
    transport_id.appendChild(doc.createTextNode('apex_transport_descriptor'))
    user_transports.appendChild(transport_id)
    rtps.appendChild(user_transports)

    # Append RTPS to participant
    participant.appendChild(rtps)
    # Append participant to profiles
    profiles.appendChild(participant)
    # Append profiles to document
    doc.appendChild(profiles)

    # Write to file
    with open(xml_file, 'w') as f:
        # Convert to string with 2-space identation,
        # not taking the first line of the XML.
        f.write(doc.toprettyxml(indent=' '*2).split('\n', 1)[1])
