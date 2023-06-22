#include "EKF.h"
#include <math.h>
#include <vector>
#include <cmath>
#include "../../lib/matrix_operations/matrix_operations.h"



double rad2deg(double rad){
    return rad/acos(-1.)*180.;
}

double deg2rad(double deg){
    return deg*acos(-1.)/180.;
}

double Euler::yaw_deg() const {
  return rad2deg(yaw);
}
double Euler::pitch_deg() const {
  return rad2deg(pitch);
}
double Euler::roll_deg() const {
  return rad2deg(roll);
}


double vector_module(const std::vector<double> &v) {
    double sum=0.0;
    int n =  v.size();

    for (int i=0; i<n; i++){
        sum +=v[i]*v[i];
    }
  return sqrt(sum);
}

std::vector <double> normalize_vector(const std::vector<double> &v) {
    double mod = vector_module(v);
    
    int n = v.size();
    std::vector<double> v_hat(n, 0.0);
    for (int i=0; i<n; i++){
       v_hat[i]=v[i]/mod;
    }
    return v_hat;
}

std::vector<std::vector<double>> getRotMatrix(const std::vector<double>& q) {
    double c00 = q[0]*q[0] + q[1] *q[1] - q[2] *q[2] - q[3] * q[3];
    double c01 = 2 * (q[1] * q[2] - q[0] * q[3]);
    double c02 = 2 * (q[1] * q[3] + q[0] * q[2]);
    double c10 = 2 * (q[1] * q[2] + q[0] * q[3]);
    double c11 = q[0]*q[0] - q[1] *q[1] + q[2] *q[2] - q[3] * q[3];
    double c12 = 2 * (q[2] * q[3] - q[0] * q[1]);
    double c20 = 2 * (q[1] * q[3] - q[0] * q[2]);
    double c21 = 2 * (q[2] * q[3] + q[0] * q[1]);
    double c22 = q[0]*q[0] - q[1] *q[1] - q[2] *q[2] + q[3] * q[3];

    std::vector<std::vector<double>> RotMat = {{c00, c01, c02},{c10, c11, c12},{c20, c21, c22}};
    return RotMat;
}

void Euler:: getEulerangles(const std::vector<double> &q){
    std::vector<std::vector<double>> M = getRotMatrix(q);
    double disc = -M[2][0];

    if (disc > 0.99999){
        yaw = 0;
        pitch = acos(-1.) / 2;
        roll = atan2(M[0][1], M[0][2]);
    }
 
    else if (disc < -0.99999){
        yaw = 0;
        pitch = -acos(-1.) / 2;
        roll = atan2(M[0][1], M[0][2]);
    }
    else{
        yaw = atan2(M[1][0], M[0][0]);
        pitch = asin(-M[2][0]);
        roll = atan2(M[2][1], M[2][2]);
    }

}

void Euler::Rot2Euler(const std::vector<std::vector<double>> &R){
    std::vector<std::vector<double>> M = R;
    double disc = -M[2][0];

    if (disc > 0.99999){
        yaw = 0;
        pitch = acos(-1.) / 2;
        roll = atan2(M[0][1], M[0][2]);
    }
 
    else if (disc < -0.99999){
        yaw = 0;
        pitch = -acos(-1.) / 2;
        roll = atan2(M[0][1], M[0][2]);
    }
    else{
        yaw = atan2(M[1][0], M[0][0]);
        pitch = asin(-M[2][0]);
        roll = atan2(M[2][1], M[2][2]);
    }

}





std::vector<std::vector<double>> EKF::getJacobianMatrix(const std::vector<double> &ref){
        double e00 = xhat_prev[0] * ref[0] + xhat_prev[3] * ref[1] - xhat_prev[2] * ref[2];
        double e01 = xhat_prev[1] * ref[0] + xhat_prev[2] * ref[1] + xhat_prev[3] * ref[2];
        double e02 = -xhat_prev[2] * ref[0] + xhat_prev[1] * ref[1] - xhat_prev[0] * ref[2];
        double e03 = -xhat_prev[3] * ref[0] + xhat_prev[0] * ref[1] + xhat_prev[1] * ref[2];    
        double e10 = -xhat_prev[3] * ref[0] + xhat_prev[0] * ref[1] + xhat_prev[1] * ref[2];
        double e11 = xhat_prev[2] * ref[0] - xhat_prev[1] * ref[1] + xhat_prev[0] * ref[2];
        double e12 = xhat_prev[1] * ref[0] + xhat_prev[2] * ref[1] + xhat_prev[3] * ref[2];
        double e13 = -xhat_prev[0] * ref[0] - xhat_prev[3] * ref[1] + xhat_prev[2] * ref[2];
        double e20 = xhat_prev[2] * ref[0] - xhat_prev[1] * ref[1] + xhat_prev[0] * ref[2];
        double e21 = xhat_prev[3] * ref[0] - xhat_prev[0] * ref[1] - xhat_prev[1] * ref[2];
        double e22 = xhat_prev[0] * ref[0] + xhat_prev[3] * ref[1] - xhat_prev[2] * ref[2];
        double e23 = xhat_prev[1] * ref[0] + xhat_prev[2] * ref[1] + xhat_prev[3] * ref[2];

        std::vector<std::vector<double>> JacobianMatrix = {{2*e00, 2*e01, 2*e02, 2*e03},{2*e10, 2*e11, 2*e12, 2*e13},{2*e20, 2*e21, 2*e22, 2*e23}};

        return JacobianMatrix;
}


std::vector<double> EKF:: predictAccelMag(){

    // Get the inverse rotation matrix
    // v_I = R * v_b
    // v_b = R^-1 * v_I
    
    std::vector<std::vector<double>> inverse_rot_mat = transposeMatrix(getRotMatrix(xhat_bar));

    // Compute the Jacobian matrix for the accelerometer ref
    std::vector<std::vector<double>> hPrime_a = getJacobianMatrix(accel_ref);
    // Compute the predicted accelerometer components in the body reference 
    // from the reference in the inertial system and the estimated rotation matrix 
    std::vector<double> accelBar = matrixVectorProduct(inverse_rot_mat, accel_ref);


    std::vector<std::vector<double>> hPrime_m = getJacobianMatrix(mag_ref);
    std::vector<double> magBar = matrixVectorProduct(inverse_rot_mat, mag_ref);

    C.resize(6, std::vector<double>(7, 0));

    for (int i=0; i<3; i++){
        for(int j=0; j<4; j++){
            C[i][j] = hPrime_a[i][j];
            C[i+3][j] = hPrime_m[i][j]; 
        }    
    }

    std::vector<double> yhat_bar = concatenateVectors(accelBar, magBar);
    return yhat_bar;

}


void EKF::getA(const double dt){

    A = {{1., 0., 0., 0., -dt/2.*(-q[1]), -dt/2.*(-q[2]), -dt/2.*(-q[3])},
         {0., 1., 0., 0.,  -dt/2.*q[0], -dt/2.*(-q[3]),  -dt/2.*q[2]},
         {0., 0., 1., 0.,  -dt/2.*q[3],  -dt/2.*q[0], -dt/2.*(-q[1])},
         {0., 0., 0., 1., -dt/2.*(-q[2]),  -dt/2.*q[1],  -dt/2.*q[0]},
         {0., 0., 0., 0.,           1.,           0.,           0.},
         {0., 0., 0., 0.,           0.,           1.,           0.},
         {0., 0., 0., 0.,           0.,           0.,           1.}};
    
}

void EKF::getB(const double dt){

    B = {{dt/2.*(-q[1]), dt/2.*(-q[2]), dt/2.*(-q[3])},
         { dt/2.*q[0], dt/2.*(-q[3]),  dt/2.*q[2]},
         { dt/2.*q[3],  dt/2.*q[0], dt/2.*(-q[1])},
         {dt/2.*(-q[2]),  dt/2.*q[1],  dt/2.*q[0]},
         {          0.,         0.,          0.},
         {          0.,         0.,          0.},
         {          0.,         0.,          0.}};
    
}


void EKF::predict(const std::vector<double> &gyro, const double dt){
    
    // Extract attitude quaternion from state vector
    for (int i=0; i<q.size(); i++){
        q[i] = xhat[i];
    }

    // q = (xhat.begin(), xhat.begin() + 4);
    // std::vector<double> q(xhat.begin(), xhat.begin() + 4);
    // printf("qhat[0] = %f\tqhat[1] = %f\tqhat[2] = %f\tqhat[3] = %f\t", q[0],q[1],q[2],q[3]);
    // printf("\n");

    // Get A matrix
    getA(dt);

    
    // Get B matrix
    getB(dt);

    

    // printMatrix(A);
    // printf("\n");
    // printMatrix(B);

    std::vector<double>term1(7,0);
    std::vector<double>term2(7,0);

    term1 = matrixVectorProduct(A, xhat);
    term2 = matrixVectorProduct(B, gyro);

    for(int i = 0; i<xhat.size(); i++){
        xhat_bar[i] = term1[i] + term2[i];
    }

    

    std::vector<double> q_bar(4, 0.0);

    for (int i=0; i<4; i++){
        q_bar[i] = xhat_bar[i];
    }

    q_bar = normalize_vector(q_bar);

    for(int i = 0; i<4; i++){
        xhat_bar[i] = q_bar[i];
        
    }

    xhat_prev = xhat;


    yhat_bar = predictAccelMag();

    // for(int i=0; i<yhat_bar.size(); i++){
    //     printf("\n%.12e\t", yhat_bar[i]);
    // }


    std::vector<std::vector<double>> term3(7,std::vector<double>(7, 0.0));

    term3 = matrixMultiplication(matrixMultiplication(A,P), transposeMatrix(A));

    // printMatrix(term3);

    for (int i= 0; i<xhat.size(); i++){
        for (int j= 0; j<xhat.size(); j++){
            P_bar[i][j] = term3[i][j] + Q[i][j]; 
        }

    }
   

}


void EKF::update(const std::vector<double> &acc,const  std::vector<double> &mag){
    
    std::vector<std::vector<double>> tmp0 = matrixMultiplication(matrixMultiplication(C, P_bar), transposeMatrix(C));
    

    std::vector<std::vector<double>> tmp1(R.size(), std::vector<double> (R.size(), 0.0));
    for (int i= 0; i<R.size(); i++){
        for (int j= 0; j<R.size(); j++){
            tmp1[i][j] = tmp0[i][j] + R[i][j]; 
        }
    }
    std::vector<std::vector<double>> k = matrixMultiplication(matrixMultiplication(P_bar, transposeMatrix(C)), matrixInverse(tmp1));

    bool ksingular = hasInfOrNaN(k);
    if (!ksingular){
        K = k;
    }


    
    std::vector<double> acc_norm = normalize_vector(acc);

    std::vector<double> mag_NED = matrixVectorProduct(getRotMatrix(xhat), mag);

    mag_NED[2] = 0;

    mag_NED = normalize_vector(mag_NED);
 
    std::vector<double> mag_B = matrixVectorProduct(transposeMatrix(getRotMatrix(xhat)), mag_NED);

    std::vector<double> measurement = concatenateVectors(acc_norm, mag_B);


 

    std::vector<double> tmp2(6,0.0);


    //  printf("\nyhat[0] = %f\tyhat[1] = %f\tyhat[2] = %f\tyhat[3] = %f\tyhat[4] = %f\t\tyhat[5] = %f\n", measurement[0],measurement[1],
    // measurement[2],measurement[3],measurement[4],measurement[5]);

    // printf("\nyhat[0] = %f\tyhat[1] = %f\tyhat[2] = %f\tyhat[3] = %f\tyhat[4] = %f\t\tyhat[5] = %f\n", yhat_bar[0],yhat_bar[1],
    // yhat_bar[2],yhat_bar[3],yhat_bar[4],yhat_bar[5]);
    

    for (int i=0; i< yhat_bar.size(); i++){
        // printf("\n%f\n", measurement[i] - yhat_bar[i] );
        tmp2[i] = measurement[i] - yhat_bar[i];
    }

    // printf("\nyhat[0] = %f\tyhat[1] = %f\tyhat[2] = %f\tyhat[3] = %f\tyhat[4] = %f\t\tyhat[5] = %f\n", tmp2[0],tmp2[1],
    // tmp2[2],tmp2[3],tmp2[4],tmp2[5]);
    


    std::vector<double> tmp3 = matrixVectorProduct(K, tmp2);



    for (int i=0; i< xhat.size(); i++){
        xhat[i] = xhat_bar[i] + tmp3[i];
    }




    for (int i=0; i<q.size(); i++){
        q[i] = xhat[i];
    }
    q = normalize_vector(q);

    for(int i = 0; i<4; i++){
        xhat[i] = q[i]; 
    }

    // for(int i=0; i<xhat.size(); i++){
    //     printf("\n%.12e\t", xhat[i]);
    // }

    std::vector<std::vector<double>> tmp4 = matrixMultiplication(K,C);
    std::vector<std::vector<double>> tmp5(7, std::vector<double>(7, 0.0));
    std::vector<std::vector<double>> eye = createDiagMatrix(7, 1.0);

    for (int i= 0; i<eye.size(); i++){
        for (int j= 0; j<eye.size(); j++){
            tmp5[i][j] = eye[i][j] - tmp4[i][j]; 
        
        }
    }

    P = matrixMultiplication(tmp5, P_bar);
    // printMatrix(P);
    
}