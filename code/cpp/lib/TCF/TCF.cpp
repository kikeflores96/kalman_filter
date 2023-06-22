#include "TCF.h"
#include <math.h>
#include <vector>
#include <cmath>
#include "../../lib/matrix_operations/matrix_operations.h"
#include "../../lib/EKF/EKF.h"
#include "../../lib/IMU/IMU.h"

double sign(const double value) {
  return ((value > 0) - (value < 0));
}

double VMOD(const std::vector<double> &v) {
    double sum=0.0;
    int n =  v.size();
    for (int i=0; i<n; i++){
        sum +=v[i]*v[i];
    }
  return sqrt(sum);
}


std::vector<double> norm_vector(const std::vector<double> &v) {
    double mod = VMOD(v);
    int n = v.size();
    std::vector<double> v_hat(n, 0.0);
    for (int i=0; i<n; i++){
       v_hat[i]=v[i]/mod;
    }
    return v_hat;
}


void vectprod(const std::vector<double> &v1, const  std::vector<double> &v2, std::vector<double> &r2) {
    r2[0] = v1[1] * v2[2] - v1[2] * v2[1];
    r2[1] = -v1[0] * v2[2] + v1[2] * v2[0];
    r2[2] = v1[0] * v2[1] - v1[1] * v2[0];
}


std::vector<std::vector<double>> triada(const std::vector<double> &v1, const std::vector<double> &v2) {
  
  std::vector<std::vector<double>> M(3,std::vector<double>(3, 0.0));
  
  M[0] = v1;

  vectprod(v1, v2, M[1]);
  M[1] = norm_vector(M[1]);

  vectprod(M[0], M[1], M[2]);
  M[2] = norm_vector(M[2]);

  M = transposeMatrix(M);

  return M;
}





std::vector<double> Rot2Quat( const std::vector<std::vector<double>> &R ){
  
  double trace = R[0][0] + R[1][1] + R[2][2]; // I removed + 1.0f; see discussion with Ethan
  std::vector<double> Q(4, 0.0);
  
  if( trace > 0 ) {// 
    double s = 0.5f / sqrtf(trace+ 1.0f);
    Q[0] = 0.25f / s;
    Q[1] = ( R[2][1] - R[1][2] ) * s;
    Q[2] = ( R[0][2] - R[2][0] ) * s;
    Q[3] = ( R[1][0] - R[0][1] ) * s;
  } else {
    if ( R[0][0] > R[1][1] && R[0][0] > R[2][2] ) {
      double s = 2.0f * sqrtf( 1.0f + R[0][0] - R[1][1] - R[2][2]);
      Q[0] = (R[2][1] - R[1][2] ) / s;
      Q[1] = 0.25f * s;
      Q[2] = (R[0][1] + R[1][0] ) / s;
      Q[3] = (R[0][2] + R[2][0] ) / s;
    } else if (R[1][1] > R[2][2]) {
      double s = 2.0f * sqrtf( 1.0f + R[1][1] - R[0][0] - R[2][2]);
      Q[0] = (R[0][2] - R[2][0] ) / s;
      Q[1] = (R[0][1] + R[1][0] ) / s;
      Q[2] = 0.25f * s;
      Q[3] = (R[1][2] + R[2][1] ) / s;
    } else {
      double s = 2.0f * sqrtf( 1.0f + R[2][2] - R[0][0] - R[1][1] );
      Q[0] = (R[1][0] - R[0][1] ) / s;
      Q[1] = (R[0][2] + R[2][0] ) / s;
      Q[2] = (R[1][2] + R[2][1] ) / s;
      Q[3] = 0.25f * s;
    }
  }

    // if (Q[0] < 0) {
    //     Q[0] = -Q[0];
    //     Q[1] = -Q[1];
    //     Q[2] = -Q[2];
    //     Q[3] = -Q[3];
    // }

  return Q;
}


void TCF::initialize(IMU &imu){

  std::vector<double>v1 = norm_vector(mag_I);
  std::vector<double>v2 = norm_vector(acc_I);

  // Triada R
  B = triada(v1, v2);

  
  
  int16_t   imusensor[3][3];
  // IMU measurements
  std::vector<double> gyro(3);    // Gyroscope p,q,r in rad/s
  std::vector<double> acc(3);     // Acceleration gx, gy, gz, in m/s^2
  std::vector<double> mag(3);     // Magnetic field in mT
  int iterations = 100;
  std::vector<double> qinit = {0,0,0,0};
  std::vector<double> k = {0,0,0};
  double theta;
  
  for (int i = 0; i < iterations; i++) {
    imu.readsensor(imusensor);
    imu.applyrange(imusensor, gyro, mag, acc);
    imu.applycalibration(gyro, acc, mag);
    TRIAD(acc, mag);
    qinit = qTriad;
    theta += 2 * acos(qinit[0]) / iterations;
    k[0]  += qinit[1] / sin(acos(qinit[0])) / iterations;
    k[1]  += qinit[2] / sin(acos(qinit[0])) / iterations;
    k[2]  += qinit[3] / sin(acos(qinit[0])) / iterations;
  }
  k = norm_vector(k);
  qinit[0] = cos(theta / 2);
  qinit[1] = k[0] * sin(theta / 2);
  qinit[2] = k[1] * sin(theta / 2);
  qinit[3] = k[2] * sin(theta / 2);

  qHat = qinit;
  qTriad_prev = qinit;
  // Euler yawpitchroll_init;

  triad_yawpitchroll.getEulerangles(qHat);

  // printf("\nyaw = %.3f\tpitch = %.3f\troll = %.3f", triad_yawpitchroll.yaw_deg(),triad_yawpitchroll.pitch_deg(),triad_yawpitchroll.roll_deg());
  // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qHat[0],qHat[1],qHat[2],qHat[3]);
  // printf("\n");

}



void TCF::TRIAD(const std::vector<double> &acc, const std::vector<double> &mag) {
  
  // Magnetic field components in the body system reference nT  
  std::vector<double> w1 = norm_vector(mag);
  // Gravity components in the inertial body reference m/s2
  std::vector<double> w2 = norm_vector(acc);
  // TRIAD algorithm
  // Triada S
  A = triada(w1, w2);
  // Matrix R=A*B'
  std::vector<std::vector<double>> R(3, std::vector<double>(3, 0.0));
  R = transposeMatrix(matrixMultiplication(A, transposeMatrix(B)));


  //float qnew[4];
  // Rotation matrix to quaternion
  // q[0] = sqrt(R[0][0] + R[1][1] + R[2][2] + 1) / 2.0;
  // q[1] = sign(R[1][2] - R[2][1]) * sqrt(R[0][0] - R[1][1] - R[2][2] + 1) / 2.0;
  // q[2] = sign(R[2][0] - R[0][2]) * sqrt(-R[0][0] + R[1][1] - R[2][2] + 1) / 2.0;
  // q[3] = sign(R[0][1] - R[1][0]) * sqrt(-R[0][0] - R[1][1] + R[2][2] + 1) / 2.0;

  qTriad = norm_vector(Rot2Quat(R));

  double dotprod = 0.0;

  for (int i=0; i<qTriad.size(); i++){
    dotprod += qTriad[i]*qTriad_prev[i];
  }

  if (dotprod<0){
    for (int i=0; i<qTriad.size(); i++){
    qTriad[i] *= -1.0;
    }
  }

  qTriad_prev = qTriad;

  triad_yawpitchroll.Rot2Euler(R);
  
  // printf("\nqhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", q[0],q[1],q[2],q[3]);
}




void TCF:: integrate(const std::vector<double> &gyro, const double dt){

  // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qHat[0],qHat[1],qHat[2],qHat[3]);
  // printf("\n");

  std::vector<std::vector<double>> Sq = {{dt/2.*(-qHat[1]), dt/2.*(-qHat[2]), dt/2.*(-qHat[3])},
                                          { dt/2.*qHat[0], dt/2.*(-qHat[3]),  dt/2.*qHat[2]},
                                          { dt/2.*qHat[3],  dt/2.*qHat[0], dt/2.*(-qHat[1])},
                                          {dt/2.*(-qHat[2]),  dt/2.*qHat[1],  dt/2.*qHat[0]}};

  std::vector<double>term1(4,0);
  term1 = matrixVectorProduct(Sq, gyro);

  for (int i=0; i<qHat.size(); i++){
    qHat_bar[i] = qHat[i] + term1[i]; 
  }
  qHat_bar = norm_vector(qHat_bar);

  // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qHat_bar[0],qHat_bar[1],qHat_bar[2],qHat_bar[3]);
  // printf("\n");

}


void TCF::combine(){

  // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qHat[0],qHat[1],qHat[2],qHat[3]);
  // printf("\n");

  // Get distance between qTriad and qHat_bar in a 4 dimension space
  double dist0 = 0;

  for(int i=0; i<qHat.size(); i++){
    dist0+= (qHat_bar[i] - qTriad[i]) * (qHat_bar[i] - qTriad[i]); 
  }
  dist0 = sqrt(dist0);

  std::vector<double> qTriad_neg = {-qTriad[0], -qTriad[1], -qTriad[2], -qTriad[3]}; 

  double dist1= 0;

  for(int i=0; i<qHat.size(); i++){
    dist1+= (qHat_bar[i] - qTriad_neg[i]) * (qHat_bar[i] - qTriad_neg[i]); 
  }
  dist1 = sqrt(dist1);

  if (dist1<dist0){
    qTriad = qTriad_neg;
  }


  double alpha = 0.9;

  for (int i=0; i<qHat.size(); i++){
    qHat[i] = alpha*qHat_bar[i] + (1 - alpha)*qTriad[i]; 
  }
  qHat = norm_vector(qHat);

double dotprod = 0.0;

  for (int i=0; i<qTriad.size(); i++){
    dotprod += qHat[i]*qHat_prev[i];
  }

  if (dotprod<0){
    for (int i=0; i<qHat.size(); i++){
    qHat[i] *= -1.0;
    }
  }

  qHat_prev = qHat;

  tcf_ypr.getEulerangles(qHat);
  
  // printf("\nqhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qHat_bar[0],qHat_bar[1],qHat_bar[2],qHat_bar[3]);
  // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qTriad[0],qTriad[1],qTriad[2],qTriad[3]);
  // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", qHat[0],qHat[1],qHat[2],qHat[3]);
  // printf("\n");

  // printf("\nyaw = %.3f\tpitch = %.3f\troll = %.3f", tcf_ypr.yaw_deg(),tcf_ypr.pitch_deg(),tcf_ypr.roll_deg());

}