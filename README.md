# uNavAHRS
uNav Attitude and Heading Reference System 7 State EKF Arduino Library.

# Description
<<<<<<< HEAD
The uNav Attitude and Heading Reference System (AHRS) is a 7 state Extended Kalman Filter (EKF) to estimate attitude and heading from IMU data. The 7 states comprise a quaternion and three gyro biases. The algorithm was developed by Jung Soon Jang at Stanford University ([1,2](<#references>)) and this library was adapted from software developed and flight tested at the [University of Minnesota UAS Research Labs](http://www.uav.aem.umn.edu), where it has been used since 2006. uNav AHRS provides good estimates of attitude and heading without requiring a GPS. It uses gyro measurements to propogate the state; accelerometers are used as a measurement update on the pitch and roll channels and magnetometers as a measurement update on the yaw channel to constrain drift.
=======
The uNav Attitude and Heading Reference System (AHRS) is a 7 state Extended Kalman Filter (EKF) to estimate attitude and heading from IMU data. The 7 states comprise a quaternion and three gyro biases. The algorithm was developed by Jung Soon Jang at Stanford University ([1,2](<#references>)) and this library was adapted from software developed and flight tested at the [University of Minnesota UAS Research Labs](http://www.uav.aem.umn.edu), where it has been used since 2006. uNav AHRS provides good estimates of attitude and heading without requiring a GPS. It uses gyro measurements to propagate the state; accelerometers are used as a measurement update on the pitch and roll channels and magnetometers as a measurement update on the yaw channel to constrain drift. Optionally, airspeed measurements may be provided to correct apparent accelerations that can lead to errors in AHRS filters used on air vehicles.
>>>>>>> 84a5d521ea8b00285e9b2d00897acfce52d094e6

# Usage

## Installation
This library requires [Eigen](https://github.com/bolderflight/Eigen) to compile. First download or clone [Eigen](https://github.com/bolderflight/Eigen) into your Arduino/libraries folder, then download or clone this library into your Arduino/libraries folder. Additionally, this library requires IMU measurements. For the included examples, an [MPU-9250 IMU](https://github.com/bolderflight/MPU9250) is used, and its library will need to be installed as well. Finally, because this library is using accelerometers and magnetometers as a measurement update, the IMU used should be well calibrated.

## Function Description

### Object Declaration
This library uses the default constructor. The following is an example of declaring a uNavAHRS object called *Filter*.

```C++
uNavAHRS Filter;
```

### Setup Functions
A sample rate divider for the magnetometer can be configured for situations where: the magnetometer is measured at a lower rate than the accelerometers and gyros or you would like the magnetometer measurement update to run at a lower rate than the rest of the filter. Additionally, the accelerometer covariance and heading angle covariance, as measured by the magnetometers, can be configured. By default, the magnetometer sample rate divider is set to 0 (i.e. magnetometers sampled at the same rate as the accelerometers and gyros), the accelerometer covariance is set to 0.96177249 (0.1g)^2 or (0.9807m/s/s)^2, the heading angle covariance is set to 0.014924 (7deg)^2 or (0.122173rad)^2.

**(optional) void setMagSrd(uint8_t magSRD)** 
This is an optional function to set the magnetometer sample rate. This is useful for situations where: the magnetometer is measured at a lower rate than the accelerometers and gyros or you would like the magnetometer measurement update to run at a lower rate than the rest of the AHRS filter. The rate is set by a sample rate divider, *uint8_t magSRD*. The rate is then given by:

*Magnetometer Rate = Filter Rate / (1 + magSRD)*

A sample rate divider of 0 means the magnetometer measurement update occurs every time the AHRS is updated. A sample rate divider of 1 is every other time and so on. A sample rate divider of 9 with an AHRS update rate of 100 Hz means the magnetometer measurement update would occur at 10 Hz. The following is an example of setting a SRD of 9.

```C++
Filter.setMagSrd(9);
```

**(optional) void setAccelCovariance(float cov)**
This is an optional function to configure the accelerometer covariance, which is a measure of the accelerometer noise. It is important to note that the covariance is not normalized and should be set to the appropriate value for the application and environment. The units are given in (m/s/s)^2. If this function is not used, a default value of 0.96177249 (0.1g)^2 or (0.9807m/s/s)^2 is applied. The following is an example of setting a covariance of 3.84708996 (0.2g)^2 or (1.9614m/s/s)^2

```C++
Filter.setAccelCovariance(3.84708996f);
```

**(optional) void setHeadingCovariance(float cov)**
This is an optional function to configure the heading covariance, which is a measure of the magnetometer noise. It is important to note that the covariance is not normalized and should be set to the appropriate value for the application and environment. The units are given in (rad)^2. If this function is not used, a default value of 0.27415570336144 (30deg)^2 or (0.5235988rad)^2 is applied. The following is an example of setting a covariance of 0.122173 (7deg)^2 or (0.122173rad)^2.

```C++
Filter.setHeadingCovariance(0.122173f);
```

### Data Collection Functions

**void update(float ias,float p,float q,float r,float ax,float ay,float az,float hx, float hy, float hz)** updates the filter with new IMU measurements, time updates propogate the state and measurement updates are made; the attitude and heading of the vehicle is updated. Inputs are:

* float p: gyro measurement in the x direction, units are rad/s.
* float q: gyro measurement in the y direction, units are rad/s.
* float r: gyro measurement in the z direction, units are rad/s.
* float ax: accelerometer measurement in the x direction, units are m/s/s.
* float ay: accelerometer measurement in the y direction, units are m/s/s.
* float az: accelerometer measurement in the z direction, units are m/s/s.
* float hx: magnetometer measurement in the x direction, units need to be consistant across all magnetometer measurements used.
* float hy: magnetometer measurement in the y direction, units need to be consistant across all magnetometer measurements used.
* float hz: magnetometer measurement in the z direction, units need to be consistant across all magnetometer measurements used.

Please note that all directional measurements (i.e. all measurements other than airspeed) need to be given in the [defined axis system](#axis-system).

```C++
// read the sensor
Imu.readSensor();
// update the filter
Filter.update(Imu.getGyroX_rads(),Imu.getGyroY_rads(),Imu.getGyroZ_rads(),Imu.getAccelX_mss(),Imu.getAccelY_mss(),Imu.getAccelZ_mss(),Imu.getMagX_uT(),Imu.getMagY_uT(),Imu.getMagZ_uT());
```

**float getRoll_rad()** returns the roll angle in units of rad.

```C++
float roll;
roll = Filter.getRoll_rad();
```

**float getPitch_rad()** returns the pitch angle in units of rad.

```C++
float pitch;
pitch = Filter.getPitch_rad();
```

**float getYaw_rad()** returns the yaw angle in units of rad.

```C++
float yaw;
yaw = Filter.getYaw_rad();
```

**float getHeading_rad()** returns the heading angle in units of rad.

```C++
float heading;
heading = Filter.getHeading_rad();
```

**float getGyroBiasX_rads** returns the current gyro bias in the x direction in units of rad/s.

```C++
float gxb;
gxb = Filter.getGyroBiasX_rads();
```

**float getGyroBiasY_rads** returns the current gyro bias in the y direction in units of rad/s.

```C++
float gyb;
gyb = Filter.getGyroBiasY_rads();
```

**float getGyroBiasZ_rads** returns the current gyro bias in the z direction in units of rad/s.

```C++
float gzb;
gzb = Filter.getGyroBiasZ_rads();
```

## <a name="axis-system"></a>Axis System
This library expects IMU data to be input in a defined axis system, which is shown below. It is a right handed coordinate system with x-axis pointed forward, the y-axis to the right, and the z-axis positive down, common in aircraft dynamics. Pitch is defined as a rotation angle around the y-axis with level as zero and roll is defined as a rotation angle around the x-axis with level as zero. Yaw is defined as a rotation angle around the z-axis with zero defined as the starting orientation. Heading is defined as a rotation angle around the z-axis with zero defined as magnetic north.

<img src="https://github.com/bolderflight/MPU9250/blob/master/docs/MPU-9250-AXIS.png" alt="Common Axis System" width="250">

## Example List
* **uNavAHRS-with-MPU9250**: demonstrates using this filter with an MPU-9250 IMU. *CalibrateMPU9250.ino* is used to calibrate the MPU-9250 IMU and store the calibration coefficients in EEPROM. *uNavAHRS_MPU9250.ino* uses the MPU-9250 IMU as measurement input to the uNav AHRS filter, which is run at a rate of 100 Hz. 

# <a name="references">References 

1. Jung Soon Jang, & Liccardo. (2006). Automation of Small UAVs using a Low Cost Mems Sensor and Embedded Computing Platform. 25th Digital Avionics Systems Conference, 2006 IEEE/AIAA, 1-9.
2. Jung Soon Jang, & Liccardo. (2007). Small UAV Automation Using MEMS. Aerospace and Electronic Systems Magazine, IEEE, 22(5), 30-34.
