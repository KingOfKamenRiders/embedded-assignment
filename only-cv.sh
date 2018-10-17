#!/bin/bash
g++ $1 -o test2 `pkg-config --cflags --libs opencv`
