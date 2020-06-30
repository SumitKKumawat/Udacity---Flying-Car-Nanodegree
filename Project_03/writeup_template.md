## Rubric Writeup:

### Rubric 1: Implemented body rate control in python and C++. The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

Error of the body rotation rates (error) by subtracting the current or estimated body rates from the desired body rates. calulating the ubar by multiplying kpPQR by the error of the body rotation rates.The controller takes into account the moments of inertia, Ix,Iy and Iz.

Calculate a desired 3-axis moment given a desired and current body rate
 
 INPUTS: 
 * pqrCmd: desired body rates [rad/s]
 * pqr: current or estimated body rates [rad/s]
 
 OUTPUT:
 return a V3F containing the desired moments for each of the 3 axes

```
 // code below for body rate control
 V3F error = pqrCmd -pqr;
 V3F ubar = kpPQR * error;
 V3F momentum = ubar * V3F(Ixx,Iyy,Izz);
    
```

### Rubric 2: Implement roll pitch control in python and C++.The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

We have implemented the roll pitch controller in C++ as per below.It takes a thrust command, x and y accelerations and the attitude of the drone (φ,ψ,θ) and outputs the p and q commands. p_c, q_c. As you can see from the implementation the mass of the drone is accounted when calculating the target angles.

Calculated acceleration by dividing the collective thrust(collThrustCmd) by the mass of the vehicle. Then need to calculate the r_x_term and r_y_term by using the same process for both. we have initialized variables like r_x_zero to r_x_six by getting their value from the rotation matrix R. The commanded r_x and r_y were calculated by taking the commanded acceleration for x and y individually and dividing them by calcuated acceleration at begining. The error for both the r_x and r_y calculated by simply subtracting the r_x_five and six respectively from the r_x and r_y commanded. At last, the pqrCmd for both x and y was found by multiplying them by the kpBank variable.


```

  // below code for role pitch control
    if (collThrustCmd > 0) {

        float acceleration = -collThrustCmd / mass;

        float r_x_zero   =   R(0, 0);
        float r_x_one    =   R(0, 1);
        float r_x_two    =   R(1, 0);
        float r_x_three  =   R(1, 1);
        float r_x_four   =   R(2, 2);
        float r_x_five   =   R(1, 2);
        float r_x_six    =   R(0, 2);

        float r_x_command = accelCmd.x / acceleration;
        float r_x_error = r_x_command - r_x_six;
        float r_x_term = kpBank * r_x_error;

        float r_y_command = accelCmd.y / acceleration;
        float r_y_error = r_y_command - r_x_five;
        float r_y_term = kpBank * r_y_error;

        float x_command = (1 / r_x_four) * (r_x_two * r_x_term - r_x_zero * r_y_term);
        float y_command = (1 / r_x_four) * (r_x_three * r_x_term - r_x_one * r_y_term);

        pqrCmd.x = x_command;
        pqrCmd.y = y_command;
        pqrCmd.z = 0.0;
    }

    else {
        pqrCmd.x = 0.0;
        pqrCmd.y = 0.0;
        pqrCmd.z = 0.0;
    }


```

### Rubric 3: Implement altitude control in c++. The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

The altitude control ensures that the vehicle stayes close to the commanded set position and velocity by computing a thrust value. The output thrust is sent to the roll pitch controller. Because the commanded thrust is going to be shared across all dimensions. The portion that points in the x,y will determine acceleration in those directions. Term integratedAltitudeError to handle the weight non-idealities.

As you can see the implementation uses both the down position and the down velocity. The output value is a thrust since it's acceleration times mass (F=ma). 

```
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

```

```
  // DescentRate and Ascent rate are constrains of the acceleration
  // we need to consider the mass of vehicle and gravity so the vehicle hovers

  float pose_err = posZCmd - posZ;

  float velocity_err = velZCmd - velZ;

  float r_three = R(2, 2);

  integratedAltitudeError = integratedAltitudeError + dt * pose_err;
  
  float u_b = kpPosZ * pose_err + kpVelZ * velocity_err + KiPosZ * integratedAltitudeError  + accelZCmd;

  float acclr =  CONSTRAIN((u_b - 9.81) / r_three, -maxDescentRate / dt, maxAscentRate / dt );

  thrust = -mass * acclr;
  
```

### Rubric 4:Implement lateral position control in C++. The controller should use the local NE position and velocity to generate a commanded local acceleration.

This controller is a PD controller in the x and y trajectories. In generates an acceleration commandin the x-y directions which is sent to the roll pitch controller.

```
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

```

```
  V3F r_position = V3F(kpPosXY, kpPosXY, 0.0);
  V3F r_velocity = V3F(kpVelXY, kpVelXY, 0.0);
  V3F velocity_command;
    
  if (velCmd.mag() > maxSpeedXY) {
      velocity_command = velCmd.norm() * maxSpeedXY;
  } else {
      velocity_command = velCmd;
  }
    
  V3F position_error = posCmd - pos;
  V3F velocity_error = velocity_command - vel;
    
  accelCmd = accelCmd + (r_position * position_error + r_velocity * velocity_error);
    
  if (accelCmd.mag() > maxAccelXY) {
       accelCmd = accelCmd.norm() * maxAccelXY;
   }
    
```

### Rubric 5:Implement yaw control in python and C++. The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

Yaw control is control through the reactive moment command and that command only effects yaw.

```
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

```

```
  // we unwrap the angle then multiply by error to get the Rate
  
  yawCmd = fmodf(yawCmd, 2 * F_PI);
  
  float yaw_err = yawCmd - yaw;
  
  yawRateCmd = kpYaw * yaw_err;

```

### Rubric 6:Implement calculating the motor commands given commanded thrust and moments in C++.The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

As you can see below the thrust and moment commands have been used to calculate the desired thrusts. To calculate the desired thrusts we have used 4 equations:

  1)collThrustCmd = f1 + f2 + f3 + f4;
  2)momentCmd.x = l * (f1 + f4 - f2 - f3); // l = L*sqrt(2)/2) - perpendicular distance to axes
  3)momentCmd.y = l * (f1 + f2 - f3 - f4);
  4)momentCmd.z = kappa * f1 - kappa * f2 + kappa * f3 - kappa * f4;
  
where torque = kappa * thrust

```
  // dividing tau values by 'l' instead of doing it in equation here, it helps in readability .....
  float l = L / sqrtf(2.f);
  float tx1 = momentCmd.x / l;
  float tx2 = momentCmd.y / l;
  float tx3 = -momentCmd.z / kappa;
  float tx4 = collThrustCmd;

  // getting below equations from the values of Ftotal, tau_x, tau_y, tau_z
	float f_1 = (tx1 + tx2 + tx3 + tx4) / 4.f;
	float f_2 = (tx4 -tx1 + tx2 - tx3 ) / 4.f;
	float f_3 = (tx1 - tx2 - tx3 + tx4) / 4.f;
	float f_4 = (tx3 + tx4 -tx1 - tx2 ) / 4.f;

  // keep in mind that, force can not be (-)ve
	cmd.desiredThrustsN[0] = f_1; // front left
	cmd.desiredThrustsN[1] = f_2; // front right
	cmd.desiredThrustsN[2] = f_3; // rear left
	cmd.desiredThrustsN[3] = f_4; // rear right
```


### Rubric 7: Your C++ controller is successfully able to fly the provided test trajectory and visually passes inspection of the scenarios leading up to the test trajectory.Ensure that in each scenario the drone looks stable and performs the required task. Specifically check that the student's controller is able to handle the non-linearities of scenario 4 (all three drones in the scenario should be able to perform the required task with the same control gains used).

```
SIMULATOR!
Select main window to interact with keyboard/mouse:
LEFT DRAG / X+LEFT DRAG / Z+LEFT DRAG = rotate, pan, zoom camera
W/S/UP/LEFT/DOWN/RIGHT - apply force
C - clear all graphs
R - reset simulation
Space - pause simulation
Simulation #1 (../config/5_TrajectoryFollow.txt)
2020-06-30 05:59:32.853555+0530 FCND-CPPSim[2770:106156] Metal API Validation Enabled
2020-06-30 05:59:32.896012+0530 FCND-CPPSim[2770:106532] flock failed to lock maps file: errno = 35
2020-06-30 05:59:32.899311+0530 FCND-CPPSim[2770:106532] flock failed to lock maps file: errno = 35
2020-06-30 05:59:32.953509+0530 FCND-CPPSim[2770:106156] [General] -[__NSCFString replaceCharactersInRange:withString:]: Range or index out of bounds
2020-06-30 05:59:32.995087+0530 FCND-CPPSim[2770:106156] [General] (
	0   CoreFoundation                      0x00007fff36bb5be7 __exceptionPreprocess + 250
	1   libobjc.A.dylib                     0x00007fff6f7f45bf objc_exception_throw + 48
	2   CoreFoundation                      0x00007fff36c6460e -[__NSCFString characterAtIndex:].cold.1 + 0
	3   CoreFoundation                      0x00007fff36c64885 -[__NSCFString replaceOccurrencesOfString:withString:options:range:].cold.1 + 0
	4   CoreFoundation                      0x00007fff36bf8908 mutateError + 44
	5   CoreFoundation                      0x00007fff36b4c4c8 -[__NSCFString replaceCharactersInRange:withString:] + 50
	6   GLUT                                0x00007fff39572851 -[GLUTApplication applicationWillFinishLaunching:] + 175
	7   CoreFoundation                      0x00007fff36b2f89f __CFNOTIFICATIONCENTER_IS_CALLING_OUT_TO_AN_OBSERVER__ + 12
	8   CoreFoundation                      0x00007fff36b2f833 ___CFXRegistrationPost1_block_invoke + 63
	9   CoreFoundation                      0x00007fff36b2f7a8 _CFXRegistrationPost1 + 372
	10  CoreFoundation                      0x00007fff36b2f414 ___CFXNotificationPost_block_invoke + 80
	11  CoreFoundation                      0x00007fff36aff58d -[_CFXNotificationRegistrar find:object:observer:enumerator:] + 1554
	12  CoreFoundation                      0x00007fff36afea39 _CFXNotificationPost + 1351
	13  Foundation                          0x00007fff39179786 -[NSNotificationCenter postNotificationName:object:userInfo:] + 59
	14  AppKit                              0x00007fff33d9f059 -[NSApplication finishLaunching] + 330
	15  AppKit                              0x00007fff340898b1 _NSApplicationBeginRunning + 179
	16  AppKit                              0x00007fff342410a5 -[NSApplication _beginRunning] + 26
	17  GLUT                                0x00007fff39572523 -[GLUTApplication run] + 117
	18  GLUT                                0x00007fff3957e7e7 glutMainLoop + 264
	19  FCND-CPPSim                         0x000000010004f79b main + 699
	20  libdyld.dylib                       0x00007fff7099bcc9 start + 1
)
2020-06-30 05:59:32.996400+0530 FCND-CPPSim[2770:106156] [General] _createMenuRef called with existing principal MenuRef already associated with menu
2020-06-30 05:59:32.998798+0530 FCND-CPPSim[2770:106156] [General] (
	0   CoreFoundation                      0x00007fff36bb5be7 __exceptionPreprocess + 250
	1   libobjc.A.dylib                     0x00007fff6f7f45bf objc_exception_throw + 48
	2   CoreFoundation                      0x00007fff36bb5a45 +[NSException raise:format:] + 189
	3   AppKit                              0x00007fff33da67d2 -[NSCarbonMenuImpl _createMenuRef] + 55
	4   AppKit                              0x00007fff33da61a2 -[NSCarbonMenuImpl _instantiateCarbonMenu] + 133
	5   AppKit                              0x00007fff33d9f1c5 -[NSApplication finishLaunching] + 694
	6   AppKit                              0x00007fff340898b1 _NSApplicationBeginRunning + 179
	7   AppKit                              0x00007fff342410a5 -[NSApplication _beginRunning] + 26
	8   GLUT                                0x00007fff39572523 -[GLUTApplication run] + 117
	9   GLUT                                0x00007fff3957e7e7 glutMainLoop + 264
	10  FCND-CPPSim                         0x000000010004f79b main + 699
	11  libdyld.dylib                       0x00007fff7099bcc9 start + 1
)
Simulation #2 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
Simulation #3 (../config/1_Intro.txt)
Simulation #4 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #5 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #6 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #7 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #8 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #9 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #10 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #11 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #12 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #13 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #14 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
Simulation #15 (../config/2_AttitudeControl.txt)
Simulation #16 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
Simulation #17 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
Simulation #18 (../config/3_PositionControl.txt)
Simulation #19 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
Simulation #20 (../config/4_Nonidealities.txt)
Simulation #21 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
Simulation #22 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
Simulation #23 (../config/5_TrajectoryFollow.txt)
Simulation #24 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
Simulation #25 (../config/X_TestManyQuads.txt)
Simulation #26 (../config/X_TestManyQuads.txt)
Simulation #27 (../config/X_TestMavlink.txt)
Simulation #28 (../config/X_TestMavlink.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```
