/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private static Joystick mechanismJoystick;

  // Select an autonomous command via Shuffleboard
  private static SendableChooser<CommandBase> aChooser;

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
    aChooser.setDefaultOption("None", null);
    aChooser.addOption("Trench Run", new TrenchRunAuto());
    aChooser.addOption("Center to Mid", new CenterMidAuto());
    aChooser.addOption("Left to Mid", new LeftMidAuto());

    // Set button binding instances
    configureButtonBindings();

    SmartDashboard.putData("Autonomous", aChooser);
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
  /* SPECIALIST ACTIONS:                                                        */
  /* Trigger - Spool Up Intake (Inject Power cells into indexer)                */
  /* Right Thumb Button - Run Indexer                                           */
  /* Right Trigger - Spool Up Turret (Shoot Power Cell)                         */
  /* Right Hatswitch - Up / Raise Generator Arm | Down / Lower Arm              */
  /*                                                                            */
  /* DRIVER ACTIONS:                                                            */
  /* Center Turret - Right Trigger                                              */
  /*----------------------------------------------------------------------------*/
  private void configureButtonBindings() {
    leftJoystick = new Joystick(Constants.joy1);
    rightJoystick = new Joystick(Constants.joy2);
    mechanismJoystick = new Joystick(Constants.joy3);

    JoystickButton shooterButton = new JoystickButton(mechanismJoystick, Constants.shooterBtn);
    shooterButton.whenHeld(new TurretClearAndFire()); // Run the turret ONLY when pressed, otherwise cancel

    JoystickButton testingShooterButton = new JoystickButton(rightJoystick, 2);
    testingShooterButton.whenHeld(new TurretClearAndFire());

    //JoystickButton homeTurret = new JoystickButton(mechanismJoystick, 11);
    //homeTurret.whenPressed(new HomeTurret());

    //JoystickButton intakeAndIndex = new JoystickButton(mechanismJoystick, 2);
    //intakeAndIndex.whenHeld(new ParallelCommandGroup(new SetIndexerPower(-1), new SetIntakePower(1)));

    JoystickButton intake = new JoystickButton(mechanismJoystick, Constants.intakeBtn);
    //intake.whenHeld(new SetIntakePower(1));

    JoystickButton driverIntake = new JoystickButton(rightJoystick, 1);
    //driverIntake.whenPressed(new PrintCommand("Driver Intake"));
    //driverIntake.whenHeld(new SetIntakePower(1));

    Trigger intakeTrigger = new Trigger(() -> intake.get() || driverIntake.get());
    intakeTrigger.whileActiveOnce(new SetIntakePower(1));

    JoystickButton outtake = new JoystickButton(mechanismJoystick, Constants.outtakeBtn);
    outtake.whenHeld(new SetIntakePower(-1));

    JoystickButton forwardIndex = new JoystickButton(mechanismJoystick, Constants.indexForward);
    forwardIndex.whenHeld(new SetIndexerPower(-1));

    JoystickButton reverseIndex = new JoystickButton(mechanismJoystick, Constants.indexReverse);
    reverseIndex.whenHeld(new ParallelCommandGroup(new EjectBallFromShooter(), new SetIndexerPower(1), new SetLoaderPower(-1)));

    JoystickButton centerTurret = new JoystickButton(leftJoystick, Constants.shooterBtn);
    centerTurret.whenHeld(new CenterTurret());
  }

  /** Return the left Joystick */
  public static Joystick getLeftJoy() {
    return leftJoystick;
  }

  /** Return the right Joystick */
  public static Joystick getRightJoy() {
    return rightJoystick;
  }

  public static Joystick getMechanismJoystick() {
    return mechanismJoystick;
  }

  /** Return the selected autonomous command  */
  public CommandBase getSelectedAuto(){
    return aChooser.getSelected();
  }
}
