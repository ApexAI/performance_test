# -*- coding: utf-8 -*-
#
# Copyright 2018 Ternaris
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
#
# SPDX-License-Identifier: Apache-2.0


"""Performance test plotter."""

from collections import OrderedDict
import io
import os
import re

from setuptools import find_packages, setup

NAME = 'apex-performance-plotter'
DESCRIPTION = 'Apex performance test report plotter'
ENTRY_POINTS = {
    'console_scripts': ['perfplot = apex_performance_plotter:plot_logfiles']
}

os.chdir(os.path.abspath(os.path.dirname(__file__)))

with io.open(os.path.join(NAME.replace('-', '_'), '__init__.py'), encoding='utf8') as f:
    VERSION = re.search(r'^__version__ = \'(.*?)\'', f.read(), flags=re.MULTILINE).group(1)

with io.open(os.path.join('README.rst'), 'rt', encoding='utf8') as f:
    README = f.read()


def read_requirements_in(path):
    """Read requirements from requirements.in file."""
    with io.open(path, 'rt', encoding='utf8') as f:  # pylint: disable=redefined-outer-name
        return [
            x.rsplit('=')[1] if x.startswith('-e') else x
            for x in [x.strip() for x in f.readlines()]
            if x
            if not x.startswith('-r')
            if not x[0] == '#'
        ]


INSTALL_REQUIRES = read_requirements_in('requirements.in')
EXTRAS_REQUIRE = {}


setup(name=NAME,
      version=VERSION,
      description=DESCRIPTION,
      long_description=README,
      author='Ternaris',
      author_email='team@ternaris.com',
      maintainer='Apex.AI',
      maintainer_email='tooling@apex.ai',
      url='https://github.com/ApexAI/performance_test',
      project_urls=OrderedDict((
          ('Code', 'https://github.com/ApexAI/performance_test'),
          ('Issue tracker', 'https://github.com/ApexAI/performance_test/issues'),
      )),
      license='Apache-2.0',
      packages=find_packages(),
      include_package_data=True,
      zip_safe=False,
      python_requires='>=3.5.2',
      install_requires=INSTALL_REQUIRES,
      extras_require=EXTRAS_REQUIRE,
      entry_points=ENTRY_POINTS)
