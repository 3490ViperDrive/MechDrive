/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;


// local imports

import frc.robot.commands.PCM_ControlCommand;



/**
 * Add your docs here.
 */
public class PCM_Subsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

    // instantiate new Solenoid controller objects
  // testing PCM here
  public Solenoid Solenoid1 = new Solenoid(1);
  public Solenoid Solenoid2 = new Solenoid(2);
  public Solenoid Solenoid3 = new Solenoid(3);
  public Solenoid Solenoid4 = new Solenoid(4);

public void ShootLeft(boolean trigger) {

  Solenoid1.set(trigger);


}

public void ShootRight(boolean trigger) {

  Solenoid2.set(trigger);


}

public void ShootUp(boolean trigger) {

  Solenoid3.set(trigger);


}

public void ShootDown(boolean trigger) {

  Solenoid4.set(trigger);


}



  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new PCM_ControlCommand());
  }
}
