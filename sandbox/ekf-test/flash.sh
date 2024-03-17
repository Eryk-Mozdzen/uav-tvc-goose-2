#!/bin/bash

file=$1

st-flash --connect-under-reset write $file 0x08000000
