## Implement Estimator
### Criteria : Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
#### MEETS SPECIFICATIONS : The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.

We have done this using the data from Graph1.txt and Graph2.txt which is placed inside log and extracted the standard devitiation(std):


### Criteria : Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.
#### MEETS SPECIFICATIONS : The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.

```
// Improve a complementary filter-type attitude filter
  // 
  // Currently a small-angle approximation integration method is implemented
  // The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers
  // 
  // Implement a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
  // to integrate the body rates into new Euler angles.
  //
  // HINTS:
  //  - there are several ways to go about this, including:
  //    1) create a rotation matrix based on your current Euler angles, integrate that, convert back to Euler angles
  //    OR 
  //    2) use the Quaternion<float> class, which has a handy FromEuler123_RPY function for creating a quaternion from Euler Roll/PitchYaw
  //       (Quaternion<float> also has a IntegrateBodyRate function, though this uses quaternions, not Euler angles)

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // thetha and sy variable declaration here
  float sy_angl = rollEst;
  float theta_angl = pitchEst;

  // Implementing a better integration method that uses the current attitude estimate (rollEst, pitchEst and ekfState(6))
  // to integrate the body rates into new Euler angles.
  Mat3x3F rot = Mat3x3F::Zeros();
  rot(0,0) = 1;
  rot(0,1) = sin(sy_angl) * tan(theta_angl);
  rot(0,2) = cos(sy_angl) * tan(theta_angl);
  rot(1,1) = cos(sy_angl);
  rot(1,2) = -sin(sy_angl);
  rot(2,1) = sin(sy_angl) / cos(theta_angl);
  rot(2,2) = cos(sy_angl) / cos(theta_angl);

  V3F angle_dot = rot * gyro;

  float predictedRoll = rollEst + dtIMU * angle_dot.x;
  float predictedPitch = pitchEst + dtIMU * angle_dot.y;

  ekfState(6) = ekfState(6) + dtIMU * angle_dot.z;

  if (ekfState(6) > F_PI) ekfState(6) -= 2.f*F_PI;

  if (ekfState(6) < -F_PI) ekfState(6) += 2.f*F_PI;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

```


### Criteria : Implement all of the elements of the prediction step for the estimator.
#### MEETS SPECIFICATIONS : The prediction step should include the state update element (`PredictState()` function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.

```
  // Predict the current state forward by time dt using current accelerations and body rates as input
  // INPUTS: 
  //   curState: starting state
  //   dt: time step to predict forward by [s]
  //   accel: acceleration of the vehicle, in body frame, *not including gravity* [m/s2]
  //   gyro: body rates of the vehicle, in body frame [rad/s]
  //   
  // OUTPUT:
  //   return the predicted state as a vector

  // HINTS 
  // - dt is the time duration for which you should predict. It will be very short (on the order of 1ms)
  //   so simplistic integration methods are fine here
  // - we've created an Attitude Quaternion for you from the current state. Use 
  //   attitude.Rotate_BtoI(<V3F>) to rotate a vector from body frame to inertial frame
  // - the yaw integral is already done in the IMU update. Be sure not to integrate it again here

  Quaternion<float> attitude = Quaternion<float>::FromEuler123_RPY(rollEst, pitchEst, curState(6));

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  // Predicting the current state forward by time dt using current accelerations and body rates as input
  predictedState(0) = curState(0) + dt * curState(3);
  predictedState(1) = curState(1) + dt * curState(4);
  predictedState(2) = curState(2) + dt * curState(5);

  V3F acc_w = attitude.Rotate_BtoI(accel);

  predictedState(3) = curState(3) + dt * acc_w.x;
  predictedState(4) = curState(4) + dt * acc_w.y;
  predictedState(5) = curState(5) + dt * acc_w.z - dt * CONST_GRAVITY;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

```

### Criteria : Implement the magnetometer update.
#### MEETS SPECIFICATIONS : The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).

```
// MAGNETOMETER UPDATE
  // Hints: 
  //  - Your current estimated yaw can be found in the state vector: ekfState(6)
  //  - Make sure to normalize the difference between your measured and estimated yaw
  //    (you don't want to update your yaw the long way around the circle)
  //  - The magnetomer measurement covariance is available in member variable R_Mag
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  zFromX(0) = ekfState(6);

  float difference = magYaw - ekfState(6);


  if ( difference > F_PI ) {
    zFromX(0) += 2.f*F_PI;

  } else if ( difference < -F_PI ) {
    zFromX(0) -= 2.f*F_PI;
  }
  
  hPrime(0, 6) = 1;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

```

### Criteria : Implement the GPS update.
#### MEETS SPECIFICATIONS : The estimator should correctly incorporate the GPS information to update the current state estimate.

```
// GPS UPDATE
  // Hints: 
  //  - The GPS measurement covariance is available in member variable R_GPS
  //  - this is a very simple update

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  zFromX(0) = ekfState(0);

  zFromX(1) = ekfState(1);

  zFromX(2) = ekfState(2);

  zFromX(3) = ekfState(3);

  zFromX(4) = ekfState(4);

  zFromX(5) = ekfState(5);



  for ( int i = 0; i < 6; i++) {
    hPrime(i,i) = 1;
  }
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

```

## Flight Evaluation

### Criteria : Meet the performance criteria of each step.
#### MEETS SPECIFICATIONS : For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.
yes, it meets all the specifications and requirements as per below output.

### Criteria : De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
#### MEETS SPECIFICATIONS : The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).
Output here. ::

```
SIMULATOR!
Select main window to interact with keyboard/mouse:
LEFT DRAG / X+LEFT DRAG / Z+LEFT DRAG = rotate, pan, zoom camera
W/S/UP/LEFT/DOWN/RIGHT - apply force
C - clear all graphs
R - reset simulation
Space - pause simulation
Simulation #1 (../config/01_Intro.txt)
2020-07-01 22:29:50.355443+0530 FCND-CPPSim[3770:115598] Metal API Validation Enabled
2020-07-01 22:29:50.384157+0530 FCND-CPPSim[3770:115936] flock failed to lock maps file: errno = 35
2020-07-01 22:29:50.384823+0530 FCND-CPPSim[3770:115936] flock failed to lock maps file: errno = 35
2020-07-01 22:29:50.414539+0530 FCND-CPPSim[3770:115598] [General] -[__NSCFString replaceCharactersInRange:withString:]: Range or index out of bounds
2020-07-01 22:29:50.416305+0530 FCND-CPPSim[3770:115598] [General] (
	0   CoreFoundation                      0x00007fff30b38be7 __exceptionPreprocess + 250
	1   libobjc.A.dylib                     0x00007fff697775bf objc_exception_throw + 48
	2   CoreFoundation                      0x00007fff30be760e -[__NSCFString characterAtIndex:].cold.1 + 0
	3   CoreFoundation                      0x00007fff30be7885 -[__NSCFString replaceOccurrencesOfString:withString:options:range:].cold.1 + 0
	4   CoreFoundation                      0x00007fff30b7b908 mutateError + 44
	5   CoreFoundation                      0x00007fff30acf4c8 -[__NSCFString replaceCharactersInRange:withString:] + 50
	6   GLUT                                0x00007fff334f5851 -[GLUTApplication applicationWillFinishLaunching:] + 175
	7   CoreFoundation                      0x00007fff30ab289f __CFNOTIFICATIONCENTER_IS_CALLING_OUT_TO_AN_OBSERVER__ + 12
	8   CoreFoundation                      0x00007fff30ab2833 ___CFXRegistrationPost1_block_invoke + 63
	9   CoreFoundation                      0x00007fff30ab27a8 _CFXRegistrationPost1 + 372
	10  CoreFoundation                      0x00007fff30ab2414 ___CFXNotificationPost_block_invoke + 80
	11  CoreFoundation                      0x00007fff30a8258d -[_CFXNotificationRegistrar find:object:observer:enumerator:] + 1554
	12  CoreFoundation                      0x00007fff30a81a39 _CFXNotificationPost + 1351
	13  Foundation                          0x00007fff330fc786 -[NSNotificationCenter postNotificationName:object:userInfo:] + 59
	14  AppKit                              0x00007fff2dd22059 -[NSApplication finishLaunching] + 330
	15  AppKit                              0x00007fff2e00c8b1 _NSApplicationBeginRunning + 179
	16  AppKit                              0x00007fff2e1c40a5 -[NSApplication _beginRunning] + 26
	17  GLUT                                0x00007fff334f5523 -[GLUTApplication run] + 117
	18  GLUT                                0x00007fff335017e7 glutMainLoop + 264
	19  FCND-CPPSim                         0x000000010005ccfb main + 699
	20  libdyld.dylib                       0x00007fff6a91ecc9 start + 1
	21  ???                                 0x0000000000000001 0x0 + 1
)
2020-07-01 22:29:50.417027+0530 FCND-CPPSim[3770:115598] [General] _createMenuRef called with existing principal MenuRef already associated with menu
2020-07-01 22:29:50.418381+0530 FCND-CPPSim[3770:115598] [General] (
	0   CoreFoundation                      0x00007fff30b38be7 __exceptionPreprocess + 250
	1   libobjc.A.dylib                     0x00007fff697775bf objc_exception_throw + 48
	2   CoreFoundation                      0x00007fff30b38a45 +[NSException raise:format:] + 189
	3   AppKit                              0x00007fff2dd297d2 -[NSCarbonMenuImpl _createMenuRef] + 55
	4   AppKit                              0x00007fff2dd291a2 -[NSCarbonMenuImpl _instantiateCarbonMenu] + 133
	5   AppKit                              0x00007fff2dd221c5 -[NSApplication finishLaunching] + 694
	6   AppKit                              0x00007fff2e00c8b1 _NSApplicationBeginRunning + 179
	7   AppKit                              0x00007fff2e1c40a5 -[NSApplication _beginRunning] + 26
	8   GLUT                                0x00007fff334f5523 -[GLUTApplication run] + 117
	9   GLUT                                0x00007fff335017e7 glutMainLoop + 264
	10  FCND-CPPSim                         0x000000010005ccfb main + 699
	11  libdyld.dylib                       0x00007fff6a91ecc9 start + 1
	12  ???                                 0x0000000000000001 0x0 + 1
)
Simulation #2 (../config/01_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #3 (../config/01_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #4 (../config/01_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #5 (../config/01_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #6 (../config/02_AttitudeControl.txt)
Simulation #7 (../config/02_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
Simulation #8 (../config/02_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
Simulation #9 (../config/03_PositionControl.txt)
Simulation #10 (../config/03_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
Simulation #11 (../config/03_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
Simulation #12 (../config/03_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
Simulation #13 (../config/04_Nonidealities.txt)
Simulation #14 (../config/04_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
Simulation #15 (../config/05_TrajectoryFollow.txt)
Simulation #16 (../config/05_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
Simulation #17 (../config/06_SensorNoise.txt)
Simulation #18 (../config/06_SensorNoise.txt)
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 67% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 68% of the time
Simulation #19 (../config/07_AttitudeEstimation.txt)
Simulation #20 (../config/07_AttitudeEstimation.txt)
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
Simulation #21 (../config/08_PredictState.txt)
Simulation #22 (../config/08_PredictState.txt)
Simulation #23 (../config/10_MagUpdate.txt)
Simulation #24 (../config/10_MagUpdate.txt)
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 62% of the time
```
