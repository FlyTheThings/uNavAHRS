/*
uNavAHRS.cpp

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

#include "Arduino.h"
#include "uNavAHRS.h"

// sets the magnetometer sample rate divider
void uNavAHRS::setMagSrd(uint8_t magSRD) {
  _magSRD = magSRD;
}

// sets the accelerometer covariance, (m/s/s)^2
void uNavAHRS::setAccelCovariance(float cov) {
  var_a = cov;
}

// sets the magnetometer heading covariance, (rad)^2
void uNavAHRS::setHeadingCovariance(float cov) {
  var_psi = cov;
}

// time and measurement update for uNavAHRS
// ias input in m/s
// p, q, r input in rad/s
// ax, ay, az input in m/s/s
// hx, hy, hz input in consistant units
void uNavAHRS::update(float ias,float p,float q,float r,float ax,float ay,float az,float hx, float hy, float hz) {
  if (!initialized) {
    // set the time
    tprev = (float) micros()/1000000.0f;
    // initial attitude and heading
    theta = asinf(ax/G);
    phi = asinf(-ay/(G*cosf(theta)));
    psi = 0;
    // magnetic heading correction due to roll and pitch angle
    Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
    Byc = hy*cosf(phi) - hz*sinf(phi);
    // finding initial heading
    if (-Byc > 0) {
      _psiInitial = PI/2.0f - atanf(Bxc/-Byc);
    } else {
      _psiInitial = 3.0f*PI/2.0f - atanf(Bxc/-Byc);
    }
    // euler to quaternion
    xs(0,0) = cosf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);  
    xs(1,0) = cosf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f) - sinf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f);
    xs(2,0) = cosf(psi/2.0f)*sinf(theta/2.0f)*cosf(phi/2.0f) + sinf(psi/2.0f)*cosf(theta/2.0f)*sinf(phi/2.0f);  
    xs(3,0) = sinf(psi/2.0f)*cosf(theta/2.0f)*cosf(phi/2.0f) - cosf(psi/2.0f)*sinf(theta/2.0f)*sinf(phi/2.0f);
    // initialization of err, measurement, and process cov. matrices
    aP(0,0)=aP(1,1)=aP(2,2)=aP(3,3)=0.1f; aP(4,4)=aP(5,5)=aP(6,6)=0.1f;
    aQ(0,0)=aQ(1,1)=aQ(2,2)=aQ(3,3)=0.0000001f; aQ(4,4)=aQ(5,5)=aQ(6,6)=0.00000000001f;
    aR(0,0)=aR(1,1)=aR(2,2)=var_a;
    initialized = true;
  } else {
    // get the change in time
    tnow = (float) micros()/1000000.0f;
    dt = 0.5f * (tnow - tprev);
    tprev = tnow;
    // gyro integration
    pc = p*dt;
    qc = q*dt;
    rc = r*dt;
    // state transition matrix
                              Fsys(0,1) = -pc;         Fsys(0,2) = -qc;         Fsys(0,3) = -rc;
    Fsys(1,0) =  pc;                                   Fsys(1,2) =  rc;         Fsys(1,3) = -qc;
    Fsys(2,0) =  qc;          Fsys(2,1) = -rc;         Fsys(2,3) =  pc;
    Fsys(3,0) =  rc;          Fsys(3,1) =  qc;         Fsys(3,2) = -pc;
    
    Fsys(0,4) = xs(1,0)*dt;   Fsys(0,5) = xs(2,0)*dt;  Fsys(0,6) = xs(3,0)*dt;
    Fsys(1,4) = -xs(0,0)*dt;  Fsys(1,5) = xs(3,0)*dt;  Fsys(1,6) = -Fsys(0,5);
    Fsys(2,4) = -Fsys(1,5);   Fsys(2,5) = Fsys(1,4);   Fsys(2,6) = Fsys(0,4);
    Fsys(3,4) = Fsys(0,5);    Fsys(3,5) = -Fsys(0,4);  Fsys(3,6) = Fsys(1,4);
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Extended Kalman filter: prediction step
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // propagation of quaternion using gyro measurement at a given sampling interval dt
    xs(0,0) += -pc*xs(1,0) - qc*xs(2,0) - rc*xs(3,0);
    xs(1,0) +=  pc*xs(0,0) - qc*xs(3,0) + rc*xs(2,0);
    xs(2,0) +=  pc*xs(3,0) + qc*xs(0,0) - rc*xs(1,0);
    xs(3,0) += -pc*xs(2,0) + qc*xs(1,0) + rc*xs(0,0); 
    // error covriance propagation: P = Fsys*P*Fsys' + Q
    aP = Fsys*aP*Fsys.transpose() + aQ;
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Extended Kalman filter: correction step for pitch and roll
    // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // nonlinear measurement equation of h_ahrs(x)
    h_ahrs(0,0) = -1.0f*G*2.0f*(xs(1,0)*xs(3,0)-xs(0,0)*xs(2,0)) + 0.05f*(ias)*q;
    h_ahrs(1,0) = -1.0f*G*2.0f*(xs(0,0)*xs(1,0)+xs(2,0)*xs(3,0)) + (ias)*r - 0.05f*(ias)*p;
    h_ahrs(2,0) = -1.0f*G*(xs(0,0)*xs(0,0)-xs(1,0)*xs(1,0)-xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)) - (ias)*q;
    // compute Jacobian matrix of h_ahrs(x)
    Hj(0,0) = G*2.0f*xs(2,0); Hj(0,1) =-1.0f*G*2.0f*xs(3,0);  Hj(0,2) = G*2.0f*xs(0,0); Hj(0,3) = -1.0f*G*2.0f*xs(1,0); Hj(0,5) = -0.05f*(ias);
    Hj(1,0) = Hj(0,3);        Hj(1,1) =-Hj(0,2);              Hj(1,2) = Hj(0,1);        Hj(1,3) = -Hj(0,0); Hj(1,4) = 0.05f*(ias); Hj(1,6) = -(ias);
    Hj(2,0) =-Hj(0,2);        Hj(2,1) =-Hj(0,3);              Hj(2,2) = Hj(0,0);        Hj(2,3) =  Hj(0,1); Hj(2,5) = (ias);
    // gain matrix aK = aP*Hj'*(Hj*aP*Hj' + aR)^-1
    aK = aP*Hj.transpose()*(Hj*aP*Hj.transpose() + aR).inverse();
    // state update
    for(size_t i=0; i < 7; i++)
    {
      xs(i,0) += aK(i,0)*(ax - h_ahrs(0,0))
              +  aK(i,1)*(ay - h_ahrs(1,0))
              +  aK(i,2)*(az - h_ahrs(2,0));
    }
    // error covariance matrix update aP = (I - aK*Hj)*aP
    aP = (Eigen::Matrix<float,7,7>::Identity() - aK*Hj)*aP;
    if(_magCount == _magSRD) { // Heading update at 10 Hz
      _magCount = 0;
      // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // second stage kalman filter update to estimate the heading angle
      // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
      // magnetic heading correction due to roll and pitch angle
      Bxc = hx*cosf(theta) + (hy*sinf(phi) + hz*cosf(phi))*sinf(theta);
      Byc = hy*cosf(phi) - hz*sinf(phi);
      // Jacobian
      Hpsi(0,0) = xs(3,0)*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*2.0f/(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))+(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))));
      Hpsi(0,1) = xs(2,0)*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*2.0f/(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))+(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))));
      Hpsi(0,2) = xs(1,0)*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*2.0f/(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))+(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))))+2.0f*xs(2,0)*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f/(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))+(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))));
      Hpsi(0,3) = xs(0,0)*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*2.0f/(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))+(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))))+2.0f*xs(3,0)*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f/(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))*2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0))+(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)))*(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))));

      // gain matrix Kpsi = aP*Hpsi'*(Hpsi*aP*Hpsi' + Rpsi)^-1
      Kpsi = aP*Hpsi.transpose()*1.0f/(Hpsi*aP*Hpsi.transpose() + var_psi);

      // state update
      psi = atan2f(2.0f*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0)),(1.0f-2.0f*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0))));
      for(size_t i=0; i < 7; i++) {
        xs(i,0) += Kpsi(i,0)*wraparound(atan2f(-Byc,Bxc) - psi);
      }

      // error covariance matrix update aP = (I - Kpsi*Hpsi)*aP
      aP = (Eigen::Matrix<float,7,7>::Identity() - Kpsi*Hpsi)*aP;
    }
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // scaling of quertonian,||q||^2 = 1
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    for(size_t i=0; i < 4; i++) {
      xs(i,0) = xs(i,0)*1.0/sqrtf(xs(0,0)*xs(0,0)+xs(1,0)*xs(1,0)+xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0));
    } 
    // obtain euler angles from quaternion
    theta = asinf(-2*(xs(1,0)*xs(3,0)-xs(0,0)*xs(2,0)));
    phi = atan2f(2*(xs(0,0)*xs(1,0)+xs(2,0)*xs(3,0)),1-2*(xs(1,0)*xs(1,0)+xs(2,0)*xs(2,0)));
    psi = atan2f(2*(xs(1,0)*xs(2,0)+xs(0,0)*xs(3,0)),1-2*(xs(2,0)*xs(2,0)+xs(3,0)*xs(3,0)));
    _magCount++;
  }
}

// returns the pitch angle, rad
float uNavAHRS::getPitch_rad() {
  return theta;
}

// returns the roll angle, rad
float uNavAHRS::getRoll_rad() {
  return phi;
}

// returns the yaw angle, rad
float uNavAHRS::getYaw_rad() {
  return psi;
}

// returns the heading angle, rad
float uNavAHRS::getHeading_rad() {
  return constrainAngle(psi+_psiInitial);
}

// returns the gyro bias estimate in the x direction, rad/s
float uNavAHRS::getGyroBiasX_rads() {
  return xs(4,0);
}

// returns the gyro bias estimate in the y direction, rad/s
float uNavAHRS::getGyroBiasY_rads() {
  return xs(5,0);
}

// returns the gyro bias estimate in the z direction, rad/s
float uNavAHRS::getGyroBiasZ_rads() {
  return xs(6,0);
}

// bound yaw angle between -180 and 180
float uNavAHRS::wraparound(float dta) {
  if(dta >  PI) dta -= (PI*2.0f);
  if(dta < -PI) dta += (PI*2.0f);
  return dta;
}

// bound heading angle between 0 and 360 
float uNavAHRS::constrainAngle(float dta){
  dta = fmod(dta,2.0f*PI);
  if (dta < 0)
    dta += 2.0f*PI;
  return dta;
}
