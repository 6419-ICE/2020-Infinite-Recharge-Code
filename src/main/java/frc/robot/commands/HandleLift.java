/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.RobotContainer.hanger;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class HandleLift extends CommandBase {
  /**
   * Creates a new ActivateLift.
   */
 

  public HandleLift() {
    addRequirements(hanger);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hanger.activateLift(Value.kReverse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.getMechanismJoystick().getY() == 1 && RobotContainer.getHangingButton().get()){
      hanger.activateLift(Value.kForward);
    } else if(RobotContainer.getMechanismJoystick().getY() == -1 && RobotContainer.getHangingButton().get()){
      hanger.activateLift(Value.kReverse);
    }
    if (RobotContainer.getMechanismJoystick().getX() == 1 && RobotContainer.getHangingButton().get()){
      hanger.positionLift(Value.kForward);
    } else if(RobotContainer.getMechanismJoystick().getX() == -1 && RobotContainer.getHangingButton().get()){
      hanger.positionLift(Value.kReverse);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
