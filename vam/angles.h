#pragma once

#include <Eigen/Dense>

enum Axis { X_AXIS, Y_AXIS };

Eigen::Vector3d
pixel_to_vect(double x, double y);

double
vect_angle(Eigen::Vector3d v, Axis axis);
