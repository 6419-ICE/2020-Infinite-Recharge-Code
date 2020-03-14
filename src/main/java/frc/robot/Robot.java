/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.HomeTurret;
import frc.robot.subsystems.Limelight;

import frc.robot.RobotContainer;

/** Hey look its a robot */
public class Robot extends TimedRobot { // CommandRobot
  private CommandBase autoCommand;
  private RobotContainer robotContainer; // Replaces OI

  /** Initialize the RobotContainer with the robot */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    //RobotContainer.compressor.setClosedLoopControl(true);
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** Called once each time the robot enters Disabled mode */
  @Override
  public void disabledInit() {
    RobotContainer.limelight.setLightMode(Limelight.LightMode.OFF);
    RobotContainer.limelight.setCameraMode(Limelight.CameraMode.VISION);
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.limelight.setLightMode(Limelight.LightMode.OFF);
    RobotContainer.limelight.setCameraMode(Limelight.CameraMode.VISION);
    //RobotContainer.limelight.setLightMode(Limelight.LightMode.ON);
    if (RobotController.getUserButton()) {
      RobotContainer.shooter.setEncoderPosition((int) Constants.Turret.TRAVERSE_SOFT_LIMIT);
    }
  }

  /** Runs an autonomous command selected in a SendableChooser */
  @Override
  public void autonomousInit() {
    RobotContainer.limelight.setLightMode(Limelight.LightMode.ON);

    RobotContainer.drivetrain.setMaxMotorSpeed(0.7);
    autoCommand = robotContainer.getSelectedAuto();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /** Called periodically during autonomous */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    RobotContainer.limelight.setLightMode(Limelight.LightMode.ON);

    RobotContainer.drivetrain.setMaxMotorSpeed(1);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (autoCommand != null) {
      autoCommand.cancel();
    }

    new HomeTurret().schedule();
  }

  /** Called periodically during operator control */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** Called periodically during test mode */
  @Override
  public void testPeriodic() {
  }
}
