#!/bin/bash

gcc `pkg-config --cflags k4a` -o calib calibration.c `pkg-config --libs k4a` -lm
