#include "stdafx.h"

#include <math.h>
#include <iostream>

#include "angles.h"

using namespace Eigen;
using namespace std;

#define M_PI 3.14159265359

// from 'CameraRig' camera.json
const double fx = 655.899779 * .5;
const double cx = 589.928903 * .5;
const double fy = 657.042298 * .5;
const double cy = 614.458172 * .5;


// from A7's camera.json
//const double fx = 869.276731;
//const double cx = 743.208037;
//const double fy = 871.118651/1.7;
//const double cy = 608.234446;


double
vect_angle(Vector3d v1, Vector3d v2)
{
//  cout << "v1 " << v1 << endl;
//  cout << "|v1| " << v1.norm() << endl;
//  cout << "v2 " << v1 << endl;
//  cout << "|v2| " << v2.norm() << endl;

//  cout << "v1 . v2 " << v1.dot(v2) << endl;

  auto cos_a = v1.dot(v2) / (v1.norm() * v2.norm());
//  cout << "cos(a)" << cos_a << endl;

  auto a = acos(cos_a); 
//  cout << a << " " << a * (360 / (M_PI * 2)) << endl;

  return a;
}


double
vect_angle(Vector3d v, Axis axis)
{
    Vector3d ax;
    ax << 0, 0, 1;

    switch (axis)
    {
        case X_AXIS:
            v[1] = 0;
            break;
        case Y_AXIS:
            v[0] = 0;
            break;
    }

//    cout << "axis " << endl << ax << endl;
//    cout << "v " << endl << v << endl;

//  cout << "v1 . v2 " << v1.dot(v2) << endl;

  auto cos_a = v.dot(ax) / (v.norm() * ax.norm());
//  cout << "cos(a) " << cos_a << endl;

  auto a = acos(cos_a); 
//  cout << a << " " << a * (360 / (M_PI * 2)) << endl;

  return a;
}



Vector3d
pixel_to_vect(double x, double y)
{
  Matrix3d cam;
  cam << fx,  0, cx,
          0, fy, cy,
          0,  0,  1;

  Vector3d v;
  v << x, y, 1;

  auto res = cam.inverse() * v;

//  cout << res << endl;

  return res;
}

Vector3d
project_vector_plane(Vector3d v, Vector3d normal)
{
// You could project your vector onto the normal vector of the plane,
// and substract the vector you've found in this way from the original vector.
    Vector3d n_proj = normal * v.dot(normal);

    return v - n_proj;
}

void
print_angles(double x, double y)
{
    Vector3d vx, vy, vz;

    vx << 1, 0, 0;
    vy << 0, 1, 0;
    vz << 0, 0, 1;

    Vector3d v = pixel_to_vect(x, y);

    cout << "x " << x << " y " << y << endl;
//    cout << v << endl;
    cout << "angle to X " << vect_angle(v, X_AXIS) * (360 / (M_PI * 2)) << endl;
//    cout << "angle to Z " << vect_angle(v, Z_AXIS) * (360 / (M_PI * 2)) << endl;
    cout << "angle to Y " << vect_angle(v, Y_AXIS) * (360 / (M_PI * 2)) << endl;
}

void
tst_proj()
{
    Vector3d foo;
    foo << 9, 9, 9;

    Vector3d norm;

    norm << 0, 1, 0;
    cout << "XZ" << endl << project_vector_plane(foo, norm) << endl;

    norm << 0, 0, 1;
    cout << "XY"  << endl << project_vector_plane(foo, norm) << endl;

    norm << 1, 0, 0;
    cout << "ZY"  << endl << project_vector_plane(foo, norm) << endl;
}

//int
//main()
//{
//tst_proj();
//    print_angles(600, 173);

//    print_angles(1018, 471);
//    print_angles(750.0, 600);


//    print_angles(1467, 872);
//    print_angles(751.0, 601);

//    print_angles(0, 1200);
//    print_angles(1500, 1200);

//    print_angles(0, 0);
//    print_angles(1500, 0);
//}
