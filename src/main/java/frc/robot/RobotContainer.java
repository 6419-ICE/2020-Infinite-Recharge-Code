/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * 6419's RobotContainer of subsystems, commands, and button mappings
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static final DriveTrain drivetrain = new DriveTrain();
  public static final Turret shooter = new Turret();
  public static final AnalogInput ultrasonic = new AnalogInput(0);
  public static final Limelight limelight = new Limelight();
  public static final Loader loader = new Loader();
  public static final Intake intake = new Intake();
  public static final Indexer indexer = new Indexer();
  // public static final DigitalInput hallEffect = new DigitalInput(1);

  private static Joystick leftJoystick;
  private static Joystick rightJoystick;


  /* Required selections for the SendableChooser */
  public enum autoSelections {
    AUTO_1, AUTO_2, AUTO_3;
  }

  // Select an autonomous command via Shuffleboard
  private static SendableChooser<autoSelections> aChooser;

  /**
   * Set default commands and construct SendableChooser for autonomous command selection
   */
  public RobotContainer() {
    // Default Commands
    drivetrain.setDefaultCommand(new HandleDriveTrain());
    shooter.setDefaultCommand(new TurretDefault());
    intake.setDefaultCommand(new HandleIntake());
    indexer.setDefaultCommand(new HandleIndexer());
    loader.setDefaultCommand(new LoaderDefault());

    /* Multiple Autonomous Selections */
    aChooser = new SendableChooser<>();
    aChooser.setDefaultOption("Default Auto", autoSelections.AUTO_1);
    aChooser.addOption("Auto 1", autoSelections.AUTO_1);
    aChooser.addOption("Auto 2", autoSelections.AUTO_2);
    aChooser.addOption("Auto 3", autoSelections.AUTO_3);

    // Set button binding instances
    configureButtonBindings();

    SmartDashboard.putData("Auto Selector", aChooser);
    SmartDashboard.putData(new syncPID());
    SmartDashboard.putData(new HomeTurret());
  }

  /*----------------------------------------------------------------------------*/
  /* ICE 6419 BUTTON MAPPINGS                                                   */
  /*                                                                            */
  /* Moving the Robot:                                                          */
  /* Right Joystick - Move Forward/Backward                                     */
  /* Left Joystick - Turn Left/Right                                            */
  /*                                                                            */
  /* Actions:                                                                   */
  /* Left Trigger - Spool Up Intake (Inject Power cells into indexer)           */
  /* Right Thumb Button - Run Indexer                                           */
  /* Right Trigger - Spool Up Turret (Shoot Power Cell)                         */
  /* Right Hatswitch - Up / Raise Generator Arm | Down / Lower Arm              */
  /*----------------------------------------------------------------------------*/
  private void configureButtonBindings() {
    leftJoystick = new Joystick(Constants.joy1);
    rightJoystick = new Joystick(Constants.joy2);

    JoystickButton shooterButton = new JoystickButton(rightJoystick, 1); // 1 = Joystick Trigger
    shooterButton.whenHeld(new TurretFire()); // Run the turret ONLY when pressed, otherwise cancel

    JoystickButton homeTurret = new JoystickButton(leftJoystick, 11);
    homeTurret.whenPressed(new HomeTurret());
  }

  /** Return the left Joystick */
  public static Joystick getLeftJoy() {
    return leftJoystick;
  }

  /** Return the right Joystick */
  public static Joystick getRightJoy() {
    return rightJoystick;
  }



  /** Return the selected autonomous command  */
  public RobotContainer.autoSelections getSelectedAuto(){
    return aChooser.getSelected();
  }
}
