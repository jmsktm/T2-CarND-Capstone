#!/bin/bash

convert -resize 400x300 -delay 20 -loop 0 images/* animation.gif
