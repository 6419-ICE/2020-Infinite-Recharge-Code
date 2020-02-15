/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.RobotContainer.shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Vision Tracking is found in the Turret subsystem, however firing Power Cells is handled here */
public class HandleTurret extends CommandBase {
  /**
   * Creates a new HandleTurret.
   */
  public HandleTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  /** Initially spool up. 
  * This will only be executed when an input is recieved from RobotContainer.configButtonBindings()
  */
  @Override
  public void initialize() {
    shooter.spoolUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  /** Stop when trigger in RobotContainer is released */
  @Override
  public void end(boolean interrupted) {
    shooter.spoolDown();
  }

  // Not used, as the command is cancelled by the user
  @Override
  public boolean isFinished() {
    return false;
  }
}
