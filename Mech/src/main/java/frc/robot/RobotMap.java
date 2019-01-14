/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  // Analog Defines

  public static int analogChan0 = 0;
  public static int analogChan1 = 1;
  public static int analogChan2 = 2;
  public static int analogChan3 = 3;

  // USB Constants 

  public static int usb_0 = 0;
  public static int usb_1 = 1;
  public static int usb_2 = 2;

  // PWM Constants

  // Control Area Network aka CAN 

  public static int motorFL = 1;
  public static int motorFR = 2;
  public static int motorBL = 3;
  public static int motorBR = 4;


  // Digital Inputs Outputs aka DIO
  public static final int addrLimitSwitch0 = 0;
  public static final int addrLimitSwitch1 = 1;


  // Joysstick Constants

  public static int addrJoyStickDriver = usb_0;
  public static int addrJoyStickAux = usb_1;

  public static int addrGreenButton = 1;
  public static int addrRedButton = 2;
  public static int addrBlueButton = 3;
  public static int addrYellowButton = 4;
  public static int addrLeftBumper = 5;
  public static int addrRightBumper = 6;
  public static int addrBackButton = 7;
  public static int addrStartButton = 8;
  public static int addrLeftStickButton = 9;
  public static int addrRightStickButton = 10;
  public static int addrLeftYaxis = 1;
  public static int addrLeftXaxis = 1;
  public static int addrRightYaxis = 2;
  public static int addrRightXaxis = 2;
  public static int addrDpadYaxis = 3;
  public static int addrDpadXaxis = 3;

  // Other Constants 

  public static float wheelDiameter = 6;
 
  // limit max throttle speed
  public static double maxMoveSpeed = 0.5;
   // limit max turn speed
  public static double maxTurnSpeed = 0.5;

  public static double invertDriveY = 1;
  public static double invertDriveX = 1;



  


  










}
