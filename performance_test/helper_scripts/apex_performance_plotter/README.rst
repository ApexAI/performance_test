Performance test plotter
========================

Performance plotter create beautiful reports from performance_test logfiles. It
is a python tool that reads the logfiles and renders PDFs using LaTeX.


Installation
------------

This package requires python3 and texlive. On an Ubuntu system you will need to
install the following packages:

::

   sudo apt-get install python3 texlive texlive-pictures texlive-luatex


Once all dependencies are in place you can setup the plotter using:

::

   pip3 install .


Using
-----

The previous step installed a ``perfplot`` binary, it can be invoked using

::
   perfplot <filename1> <filename2> ...

Be sure to also check ``perfplot -h`` for additional options.
