# Copyright 2019 Apex.AI, Inc.
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

from apex_performance_plotter.generate_plots import get_range
from apex_performance_plotter.generate_plots import generate_figures


def test_get_range():
    assert get_range(1, [1, 2, 3, 4, 5]) == [0, 6]
    assert get_range(1, [1, 2, 3], [4, 5]) == [0, 6]
    assert get_range(1, [1, 5], [2, 3, 4]) == [0, 6]


def test_get_range_asymetric():
    # Check that we can have different padding on the left vs right
    assert get_range((1, 2), [1, 2, 3, 4, 5]) == [0, 7]


def test_generate_figures_empty_dataset():

    generate_figures([], [], [], [], [], [], [], [], [])
