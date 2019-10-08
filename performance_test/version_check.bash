#!/bin/bash

REV=$(git rev-parse --short HEAD)
if [[ `git status --porcelain` ]]; then
  REV="$REV-dirty"
fi
printf $REV
