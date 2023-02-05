#ifndef ATTITUDE_H_
#define ATTITUDE_H_
#include <Eigen/Core>
#include <math.h>

using namespace Eigen;

class attitude{
  private:
      Eigen::Array<double, 4, 1> q {1.0, 0.0 ,0.0, 0.0};
      Eigen::Array<double, 3, 1> bias {0.0, 0.0, 0.0};
      Eigen::Array<double, 7, 1> xhat;
      Eigen::Matrix<double, 7, 7> P =  0.01*Matrix<double, 7, 7>::Identity();
      Eigen::Matrix<double, 7, 7> Q = 0.001*Matrix<double, 7, 7>::Identity();
      Eigen::Matrix<double, 6, 6> R =   0.1*Matrix<double, 6, 6>::Identity();
      Eigen::Matrix<double, 7, 7> K;
      Eigen::Matrix<double, 7, 7> A;
      Eigen::Matrix<double, 6, 6> B;
      Eigen::Matrix<double, 6, 6> C;
      Eigen::Array<double, 7, 1> xhat_bar;
      Eigen::Array<double, 7, 1> xhat_prev;
      Eigen::Array<double, 7, 1> pbar;
      Eigen::Array<double, 3, 1> accel_ref {0.0, 0.0, -1.0};

  public:


};

#endif /* ATTITUDE_H_ */



    // double xHat[7];
    // double quaternion[4] {1.0, 0.0, 0.0, 0.0};     // Initial estimate of the quaternion
    // double bias[3] = {0.0, 0.0, 0.0};              // Initial estimate of the gyro bias

    // double xHat[7], xHatBar[7], xHatPrev[7];
    // double yHatBar[3];
    // double P[7][7],PBar[7][7], Q[7][7], R[6][6];
    // double K;
    // double A;
    // double B;
    // double C;
    // double accelReference[3] = {0., 0., -1.};
    // double magReference[3] = {0., -1., 0.};
    // double mag_Ainv[3][3] = {{4.34883093e-03, -4.85734863e-05,  2.71406130e-04},
    //                         {-4.85734863e-05,  4.10542322e-03, -6.63845234e-05},
    //                         {2.71406130e-04, -6.63845234e-05,  4.34471427e-03}};
    // double mag_b[3] = {7.43143287, -95.03272074, 397.19236686};
    // self.p = np.identity(7) * 0.01
    // self.Q = np.identity(7) * 0.001
    // self.R = np.identity(6) * 0.1
    // self.K = None
    // self.A= None
    // self.B = None
    // self.C = None
    // self.pBar = None