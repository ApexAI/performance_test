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


"""Tool for plotting performance test reports."""

import itertools
import os
import re
import shutil
import subprocess
import sys
import tempfile

import click

import jinja2

import pandas

from .generate_plots import generate_figures


__version__ = '0.1.0'


def sanitize(val):
    """Escape latex specical characters in string values."""
    if isinstance(val, str):
        val = val.replace('_', r'\_').replace('%', r'\%')
    return val


def load_logfile(filename):
    """Load logfile into header dictionary and pandas dataframe."""
    with open(filename) as source:
        header = {
            item.split(':')[0].strip(): item.split(':', maxsplit=1)[1].strip()
            for item in itertools.takewhile(lambda x: not x.startswith('---'), source)
        }
        dataframe = pandas.read_csv(source, sep='[ \t]*,[ \t]*', engine='python')
        unnamed = [col for col in dataframe.keys() if col.startswith('Unnamed: ')]
        if unnamed:
            dataframe.drop(unnamed, axis=1, inplace=True)
    return header, dataframe


def load_template():
    """Load the jinja template in an appropriate environment."""
    latex_jinja_env = jinja2.Environment(
        block_start_string=r'\blk{',
        block_end_string='}',
        variable_start_string=r'\var{',
        variable_end_string='}',
        trim_blocks=True,
        autoescape=False,
        loader=jinja2.FileSystemLoader(os.path.abspath(os.path.dirname(__file__)))
    )
    latex_jinja_env.filters.update({
        'sanitize': sanitize,
        'legend_entries': lambda x: ','.join(sanitize(y['name'])
                                             for y in x['traces'] + x['axis2']['traces'])
    })
    return latex_jinja_env.get_template('template.tex')


def create_kv(dct, key, boolish=False):
    """Create a key-value item from a metadata entry."""
    if boolish:
        value = bool(int(sanitize(dct[key])))
        if key.startswith('Not'):
            value = not value
            key = key[4:]
        first = key.split(' ')[0].lower()
        if first in ['use', 'using']:
            key = key[len(first) + 1:]
        key = key[0].upper() + key[1:]
        return {'key': sanitize(key), 'value': '*' if value else '-'}
    return {'key': sanitize(key), 'value': sanitize(dct[key])}


def create_layout(header, dataframe):
    """Create a rendering context for the jinja template."""
    header_fields = {
        'Logfile name', 'Experiment id', 'Communication mean', 'Publishing rate',
        'Topic name', 'Number of publishers', 'Number of subscribers', 'Maximum runtime (sec)',
        'DDS domain id', 'QOS', 'Use ros SHM', 'Use single participant', 'Not using waitset',
        'Not using Connext DDS Micro INTRA', 'Performance Test Version',
    }

    header.update(dict('QOS {}'.format(x).split(': ')
                       for x in re.findall(r'(?:(.+?: \S+)\s?)', header['QOS'])))

    xaxis = dataframe.T_experiment.tolist()
    y11 = dataframe['latency_min (ms)'].tolist()
    y12 = dataframe['latency_max (ms)'].tolist()
    y13 = dataframe['latency_mean (ms)'].tolist()
    y14 = dataframe['latency_variance (ms)'].tolist()

    yr11 = (dataframe['ru_maxrss'] / 1e3).tolist()

    y21 = dataframe['ru_minflt'].tolist()
    y22 = dataframe['ru_majflt'].tolist()
    y23 = dataframe['ru_nivcsw'].tolist()
    y24 = dataframe['cpu_usage (%)'].tolist()

    means = dataframe.mean().round(4)

    return {
        'title': 'Performance',
        'quickrefs': [
            create_kv(header, 'Logfile name'),
            create_kv(header, 'Experiment id'),
            create_kv(header, 'Communication mean'),
        ],
        'figures': generate_figures(
            xaxis,
            y11,
            y12,
            y13,
            y14,
            yr11,
            y21,
            y22,
            y23,
            y24
        ),
        'categories': [
            {'name': 'test setup', 'items': [
                create_kv(header, 'Performance Test Version'),
                create_kv(header, 'Publishing rate'),
                create_kv(header, 'Topic name'),
                create_kv(header, 'Number of publishers'),
                create_kv(header, 'Number of subscribers'),
                create_kv(header, 'Maximum runtime (sec)'),
                create_kv(header, 'DDS domain id'),
                create_kv(header, 'Use ros SHM', boolish=True),
                create_kv(header, 'Use single participant', boolish=True),
                create_kv(header, 'Not using waitset', boolish=True),
                create_kv(header, 'Not using Connext DDS Micro INTRA', boolish=True),
            ]},
            {'name': 'average results', 'items': [
                {'key': 'Experiment Status', 'value': 'success' if xaxis else 'failed'},
                *[create_kv(means, key) for key in means.keys()
                  if key != 'T_experiment' and not key.startswith('ru_')],
            ]},
            {'name': 'environment', 'items': [
                *[create_kv(header, key) for key in sorted(set(header.keys()) - header_fields)],
            ]},
        ],
    }


def render(template, filepath, skip_head=0, skip_tail=0):
    """Render one file into a pdf."""
    filename = os.path.basename(filepath)
    header, dataframe = load_logfile(filepath)

    # Valudate the dataframe.  We are expected to produce a plot if there is no data, but if the
    # user gave us silly command line arguments like "skip more data than we recorded" we can
    # still error out
    if skip_head + skip_tail > len(dataframe):
        print(
            "ERROR: Told to skip {} rows from the start and {} rows from the end "
            "but there are only {} rows or data".format(
                skip_head,
                skip_tail,
                len(dataframe)
            )
        )
        sys.exit(1)

    dataframe.drop(dataframe.index[0:skip_head], inplace=True)
    dataframe.drop(dataframe.index[len(dataframe) - skip_tail:len(dataframe)], inplace=True)
    layout = create_layout(header, dataframe)
    tex = template.render(layout)

    with tempfile.TemporaryDirectory() as dirname:
        texname = os.path.join(dirname, '{}.tex'.format(filename))
        with open(texname, 'w') as fhandle:
            fhandle.write(tex)

        print('Running tex for {}'.format(filepath))
        cmd = ['lualatex', '--interaction=nonstopmode', texname]
        ret = subprocess.run(cmd, cwd=dirname, stdout=subprocess.PIPE)
        if ret.returncode:
            print(ret.stdout.decode('ascii'))
            print('ERROR: Could not compile tex file.')
            sys.exit(2)

        pdfname = os.path.join(dirname, '{}.pdf'.format(filename))
        shutil.copy(pdfname, os.path.dirname(os.path.abspath(filepath)))


@click.command()
@click.option('--skip-head', type=click.IntRange(0, float('inf')),
              default=0, help='Number of head rows to skip.')
@click.option('--skip-tail', type=click.IntRange(0, float('inf')),
              default=0, help='Number of tail rows to skip.')
@click.argument('filenames', type=click.Path(exists=True), nargs=-1, required=True)
def plot_logfiles(skip_head, skip_tail, filenames):
    """CLI entrypoint for plotting multiple files."""
    if not shutil.which('lualatex'):
        print('This tool requires lualatex, please install texlive.')
        sys.exit(1)

    template = load_template()
    for filename in filenames:
        abs_path = os.path.abspath(filename)
        render(template, abs_path, skip_head, skip_tail)


if __name__ == '__main__':
    plot_logfiles()  # pylint: disable=no-value-for-parameter
