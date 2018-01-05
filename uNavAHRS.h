/*
uNavAHRS.h

Original Author: 
Jung Soon Jang
2006-08-31
University of Minnesota 
Aerospace Engineering and Mechanics 
Copyright 2011 Regents of the University of Minnesota. All rights reserved.

Updated to be a class, use Eigen, and compile as an Arduino library.
Added methods to set covariance and get gyro bias. Added initialization to 
estimated angles rather than assuming IMU is level. Added method to get
magnetic heading rather than just psi:
Brian R Taylor
brian.taylor@bolderflight.com
2017-12-14
Bolder Flight Systems
Copyright 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef UNAVAHRS_h
#define UNAVAHRS_h

#include "Arduino.h"
#include "Eigen.h"
#include <Eigen/Dense>

class uNavAHRS {
	public:
    void setMagSrd(uint8_t magSRD);
    void setAccelCovariance(float cov);
    void setHeadingCovariance(float cov);
    void update(float p,float q,float r,float ax,float ay,float az,float hx, float hy, float hz);
    float getPitch_rad();
    float getRoll_rad();
    float getYaw_rad();
    float getHeading_rad();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
  private:
    // err covariance of accelerometers
    float var_a = 0.96177249f; //(0.1*g)^2
    // err covariance of magnetometer heading
    float var_psi = 0.014924; //(7*d2r)^2
    // estimated attitude
    float phi, theta, psi;
    // acceleration due to gravity
    const float G = 9.807f;
    const float G2 = 2.0f*9.807f;
    // initialized
    bool initialized = false;
    // initial heading
    float _psiInitial;
    // EKF state vector
    Eigen::Matrix<float,7,1> xs = Eigen::Matrix<float,7,1>::Zero();
    // timing
    float tnow, tprev, dt;
    // gyro integration
    float pc, qc, rc;
    // magnetometer skip factor
    uint8_t _magSRD = 0;
    uint8_t _magCount = 0;
    // accelerometer skip factor
    uint8_t _accelSRD = 1;
    uint8_t _accelCount = 0;
    // err, measurement, and process cov. matrices
    Eigen::Matrix<float,7,7> aP = Eigen::Matrix<float,7,7>::Zero();
    Eigen::Matrix<float,7,7> aQ = Eigen::Matrix<float,7,7>::Zero();
    Eigen::Matrix<float,3,3> aR = Eigen::Matrix<float,3,3>::Zero();
    // gain matrix
    Eigen::Matrix<float,7,3> aK = Eigen::Matrix<float,7,3>::Zero();
    // state transition matrix
    Eigen::Matrix<float,7,7> Fsys = Eigen::Matrix<float,7,7>::Zero();
    // measurement update
    Eigen::Matrix<float,3,1> h_ahrs = Eigen::Matrix<float,3,1>::Zero();
    // Jacobian matrix
    Eigen::Matrix<float,3,7> Hj = Eigen::Matrix<float,3,7>::Zero();
    // heading matrices
    Eigen::Matrix<float,1,7> Hpsi = Eigen::Matrix<float,1,7>::Zero();
    Eigen::Matrix<float,7,1> Kpsi = Eigen::Matrix<float,7,1>::Zero();
    // magnetic heading corrected for roll and pitch angle
    float Bxc, Byc;
    // bound yaw angle between -180 and 180
    float wraparound(float dta);
    // bound heading angle between 0 and 360
    float constrainAngle(float dta);
};

#endif
