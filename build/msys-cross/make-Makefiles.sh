#!/bin/sh
# Run this from within a MSYS sh shell
cmake -G "MSYS Makefiles" ../../source && cmake-gui ../../source
