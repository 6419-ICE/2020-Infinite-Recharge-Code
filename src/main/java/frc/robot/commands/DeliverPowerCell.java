/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.shooter;

public class DeliverPowerCell extends CommandBase {
  
  /* TODO: Implement DeliverPowerCell for Auto */
  public DeliverPowerCell() {
    addRequirements(shooter);
  }

  // Spool up on start of Auto
  @Override
  public void initialize() {
    shooter.spoolUp();
  }

  @Override
  public void execute() { 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spoolDown();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // return index is empty
  }
}
