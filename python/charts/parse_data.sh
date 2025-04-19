#!/bin/bash

grep Impact_ python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/impact_data_$1.txt
grep time python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/time_$1.txt
grep yaw_r python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/yaw_r_$1.txt
grep yaw: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/yaw_$1.txt
grep pitch: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/pitch_$1.txt
grep pitch_r: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/pitch_r_$1.txt
grep roll: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/roll_$1.txt
grep roll_r: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/roll_r_$1.txt
grep  Position: python/charts/text/data_$1.txt | grep [0-9] | grep -v ',$' >python/charts/text/position_$1.txt
grep Circles: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/circles_$1.txt
grep Velocity: python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/velocity_$1.txt
grep initial_ python/charts/text/data_$1.txt | grep [0-9] >python/charts/text/initial_conditions_$1.txt