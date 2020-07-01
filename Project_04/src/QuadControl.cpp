#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();
  
  // variables needed for integral control
  integratedAltitudeError = 0;
  
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
  
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
  
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);
  
  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);
  
  kpPQR = config->Get(_config + ".kpPQR", V3F());
  
  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);
  
  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);
  
  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
  
  
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to
  //   individual motor thrust commands
  // INPUTS:
  //   desCollectiveThrust: desired collective thrust [N]
  //   desMoment: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
  
  // HINTS:
  // - you can access parts of desMoment via e.g. desMoment.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  //float l = L / sqrtf(2.f);
  //float t1 = momentCmd.x / l;
  //float t2 = momentCmd.y / l;
  //float t3 = - momentCmd.z / kappa;
  //float t4 = collThrustCmd;
  
  //cmd.desiredThrustsN[0] = (t1 + t2 + t3 + t4)/4.f;  // front left  - f1
  //cmd.desiredThrustsN[1] = (-t1 + t2 - t3 + t4)/4.f; // front right - f2
  //cmd.desiredThrustsN[2] = (t1 - t2 - t3 + t4)/4.f ; // rear left   - f4
  //cmd.desiredThrustsN[3] = (-t1 - t2 + t3 + t4)/4.f; // rear right  - f3

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
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS:
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes
  
  // HINTS:
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)
  
  V3F momentCmd;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //V3F I;
  //I.x = Ixx;
  //I.y = Iyy;
  //I.z = Izz;
  //momentCmd = I * kpPQR * ( pqrCmd - pqr );

  // code below for body rate control
  V3F error = pqrCmd -pqr;
  V3F ubar = kpPQR * error;
  V3F momentum = ubar * V3F(Ixx,Iyy,Izz);
  


  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return momentum;
}

// returns a desired roll and pitch rate
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS:
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)
  
  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first
  
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //if ( collThrustCmd > 0 ) {
  //  float c = - collThrustCmd / mass;
  //  float b_x_cmd = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
  //  float b_x_err = b_x_cmd - R(0,2);
  //  float b_x_p_term = kpBank * b_x_err;
    
  //  float b_y_cmd = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
  //  float b_y_err = b_y_cmd - R(1,2);
  //  float b_y_p_term = kpBank * b_y_err;
    
  //  pqrCmd.x = (R(1,0) * b_x_p_term - R(0,0) * b_y_p_term) / R(2,2);
  //  pqrCmd.y = (R(1,1) * b_x_p_term - R(0,1) * b_y_p_term) / R(2,2);
  //} else {
  //  pqrCmd.x = 0.0;
  //  pqrCmd.y = 0.0;
  //}
  
  //pqrCmd.z = 0;


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

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
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
  
  // HINTS:
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER
  
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //float z_err = posZCmd - posZ;
  //float p_term = kpPosZ * z_err;
  
  //float z_dot_err = velZCmd - velZ;
  //integratedAltitudeError += z_err * dt;
  
  
  //float d_term = kpVelZ * z_dot_err + velZ;
  //float i_term = KiPosZ * integratedAltitudeError;
  //float b_z = R(2,2);
  
  //float u_1_bar = p_term + d_term + i_term + accelZCmd;
  
  //float acc = ( u_1_bar - CONST_GRAVITY ) / b_z;
  
  //thrust = - mass * CONSTRAIN(acc, - maxAscentRate / dt, maxAscentRate / dt);


  // DescentRate and Ascent rate are constrains of the acceleration
  // we need to consider the mass of vehicle and gravity so the vehicle hovers

  float pose_err = posZCmd - posZ;

  float velocity_err = velZCmd - velZ;

  float r_three = R(2, 2);

  integratedAltitudeError = integratedAltitudeError + dt * pose_err;
  
  float u_b = kpPosZ * pose_err + kpVelZ * velocity_err + KiPosZ * integratedAltitudeError  + accelZCmd;

  float acclr =  CONSTRAIN((u_b - 9.81) / r_three, -maxDescentRate / dt, maxAscentRate / dt );

  thrust = -mass * acclr;

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmd)
{
  // Calculate a desired horizontal acceleration based on
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS:
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmd: desired acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations.
  //     the Z component should be 0
  // HINTS:
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you cap the horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY
  
  // make sure we don't have any incoming z-component
  accelCmd.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;
  
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  //V3F kpPos;
  //kpPos.x = kpPosXY;
  //kpPos.y = kpPosXY;
  //kpPos.z = 0.f;
  
  //V3F kpVel;
  //kpVel.x = kpVelXY;
  //kpVel.y = kpVelXY;
  //kpVel.z = 0.f;
  
  //V3F capVelCmd;
  //if ( velCmd.mag() > maxSpeedXY ) {
  //  capVelCmd = velCmd.norm() * maxSpeedXY;
  //} else {
  //  capVelCmd = velCmd;
  //}
  
  //accelCmd = kpPos * ( posCmd - pos ) + kpVel * ( capVelCmd - vel ) + accelCmd;
  
  //if ( accelCmd.mag() > maxAccelXY ) {
  //  accelCmd = accelCmd.norm() * maxAccelXY;
  //}

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
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS:
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS:
  //  - use fmodf(foo,b) to constrain float foo to range [0,b]
  //  - use the yaw control gain parameter kpYaw
  
  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  

  // we unwrap the angle then multiply by error to get the Rate
  float yaw_cmd_second = 0;
  if ( yawCmd > 0 ) {
    yaw_cmd_second = fmodf(yawCmd, 2 * F_PI);
  } else {
    yaw_cmd_second = -fmodf(-yawCmd, 2 * F_PI);
  }


  float yaw_error = yaw_cmd_second - yaw;
  if ( yaw_error > F_PI ) {
    yaw_error -= 2 * F_PI;
  } if ( yaw_error < -F_PI ) {
    yaw_error += 2 * F_PI;
  }
  yawRateCmd = kpYaw * yaw_error;
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return yawRateCmd;
  
}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);
  
  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);
  
  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());
  
  V3F desMoment = BodyRateControl(desOmega, estOmega);
  
  return GenerateMotorCommands(collThrustCmd, desMoment);
}
