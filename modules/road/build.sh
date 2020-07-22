#!/bin/sh
cd ~/godot-stable
scons platform=x11 target=release_debug -j8
