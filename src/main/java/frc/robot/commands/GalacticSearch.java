// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TrajectoryAttempt;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.limelight;
import static frc.robot.RobotContainer.intake;

import java.nio.file.PathMatcher;;


public class GalacticSearch extends CommandBase {
  private String pathColor;
  private String pathName;
  private CommandBase trajectoryCommand;
  private CommandBase intakeCommand;
  /** Creates a new GalacticSearch. */
  public GalacticSearch(String pathName) {
    addRequirements(drivetrain, intake, limelight);
    this.pathName=pathName;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathColor = limelight.canSeeTarget() ? "Red" : "Blue";
    trajectoryCommand = new TrajectoryAttempt(pathName + pathColor);
    intakeCommand = new SetIndexerPower(-1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trajectoryCommand.schedule();
    intakeCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
