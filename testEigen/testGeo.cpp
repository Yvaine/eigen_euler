#define _USE_MATH_DEFINES
#include <cmath>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

using namespace Eigen;
using namespace std;

void eulerAndRotation();

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

int main() {
  float angle1 = 30 * M_PI / 180;
  float angle2 = 90 * M_PI / 180;
  float angle3 = 120 * M_PI / 180;
  Eigen::Affine3d r = create_rotation_matrix(angle1, angle2, angle3);
  Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(1,1,2)));

  Eigen::Matrix4d m = (t * r).matrix(); // Option 1
  Eigen::Matrix3d rotation = r.linear();
  rotation.eulerAngles(0,1,2);
  Eigen::Vector3d translate = r.translation();
  //Eigen::Matrix4d m = t.matrix(); // Option 2
  m *= r.matrix();

  eulerAndRotation();
  return 0;
}

void eulerAndRotation()
{
	Matrix3f mTp;

    mTp = AngleAxisf(0.5*M_PI, Vector3f::UnitX())
      * AngleAxisf(0.25*M_PI, Vector3f::UnitY())
      * AngleAxisf(0.6*M_PI, Vector3f::UnitZ());

    std::cout << "original rotation:" << endl;
    cout << mTp << endl << endl;

    Vector3f ea = mTp.eulerAngles(0, 1, 2); 
    cout << "to Euler angles:" << endl;
    cout << ea << endl << endl;

    Matrix3f n;
    n = AngleAxisf(ea[0], Vector3f::UnitX())
       *AngleAxisf(ea[1], Vector3f::UnitY())
       *AngleAxisf(ea[2], Vector3f::UnitZ()); 
    cout << "recalc original rotation:" << endl;
    cout << n << endl;
}