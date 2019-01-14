/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import frc.robot.RobotMap;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  //Joystick stick = new Joystick(port);

    public Joystick controllerDrive = new Joystick(RobotMap.addrJoyStickDriver);
  //Button button = new JoystickButton(stick, buttonNumber);    
/*
  public Button iGreenButton = new JoystickButton(controllerDrive, RobotMap.addrGreenButton);
  public Button iRedButton = new JoystickButton(controllerDrive, RobotMap.addrRedButton);
  public Button iBlueButton = new JoystickButton(controllerDrive, RobotMap.addrBlueButton);
  public Button iYellowButton = new JoystickButton(controllerDrive, RobotMap.addrYellowButton);
  public Button iLeftBumper = new JoystickButton(controllerDrive, RobotMap.addrLeftBumper);
  public Button iRightBumper = new JoystickButton(controllerDrive, RobotMap.addrRightBumper);
  public Button iBackButton = new JoystickButton(controllerDrive, RobotMap.addrBackButton);
  public Button iStartButton = new JoystickButton(controllerDrive, RobotMap.addrStartButton);
  public Button iLeftStickButton = new JoystickButton(controllerDrive, RobotMap.addrLeftStickButton);
  public Button iRightStickButton = new JoystickButton(controllerDrive, RobotMap.addrRightStickButton);
*/

/*
    public static int addrLeftYaxis = 1;
    public static int addrLeftXaxis = 1;
    public static int addrRightYaxis = 2;
    public static int addrRightXaxis = 2;
    public static int addrDpadYaxis = 3;
    public static int addrDpadXaxis = 3;
*/

    public Joystick controllerAux = new Joystick(RobotMap.addrJoyStickAux);

  //Button button = new JoystickButton(stick, buttonNumber);    



    // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
