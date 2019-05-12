#!/bin/bash

convert -resize 400x300 -delay 50 -loop 0 images/* animation.gif
