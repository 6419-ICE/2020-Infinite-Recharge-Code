// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TrajectoryAttempt extends CommandBase {

  String trajectoryJSON;
  String path;
  Trajectory trajectory;
  Path trajectoryPath;
  RamseteCommand ramseteCommand;

  /** Creates a new TrajectoryAttempt. */
  public TrajectoryAttempt(String path) {
    addRequirements(RobotContainer.drivetrain);
    this.path = path;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        trajectoryJSON = "paths/" + this.path + ".wpilib.json";

        trajectory = new Trajectory();
        try {
          trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
          // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
          // ex.getStackTrace());
        }

        ramseteCommand = new RamseteCommand(trajectory, RobotContainer.drivetrain::getPose,
        new RamseteController(Constants.Drivetrain.kRamseteB, Constants.Drivetrain.kRamseteZeta),
        (new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts, Constants.Drivetrain.ksVoltsSecondsPerMeter,
            Constants.Drivetrain.ksVoltsSecondsSquaredPerMeter)),
        Constants.Drivetrain.kDriveKinematics, RobotContainer.drivetrain::getWheelSpeeds,
        new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
        new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        RobotContainer.drivetrain::tankDriveVolts, RobotContainer.drivetrain);
        // Reset odometry to the starting pose of the trajectory.
        RobotContainer.drivetrain.resetOdometry(trajectory.getInitialPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ramseteCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
