# Navigation

This node provides an action server that receives a goal pose and navigates to it.

## Location data

The location data is, for now, provided by the _odometry_. This should at some point change to the _localization_ package.

## Motor controll

The motors are controlled via the _cartesian controller_ package which accepts linear and angular velocities.
