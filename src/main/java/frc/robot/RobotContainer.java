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
  private static Joystick mechanismJoystick;

  private static final I2C.Port i2cPort = I2C.Port.kOnboard;
  public static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

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
  /* Actions:              
  */
  /* Left Trigger - Spool Up Intake (Inject Power cells into indexer)           */
  /* Right Thumb Button - Run Indexer                                           */
  /* Right Trigger - Spool Up Turret (Shoot Power Cell)                         */
  /* Right Hatswitch - Up / Raise Generator Arm | Down / Lower Arm              */
  /*----------------------------------------------------------------------------*/
  private void configureButtonBindings() {
    leftJoystick = new Joystick(Constants.joy1);
    rightJoystick = new Joystick(Constants.joy2);
    mechanismJoystick = new Joystick(2);

    JoystickButton shooterButton = new JoystickButton(mechanismJoystick, 1); // 1 = Joystick Trigger
    shooterButton.whenHeld(new TurretFire()); // Run the turret ONLY when pressed, otherwise cancel

    //JoystickButton homeTurret = new JoystickButton(mechanismJoystick, 11);
    //homeTurret.whenPressed(new HomeTurret());

    JoystickButton intakeAndIndex = new JoystickButton(mechanismJoystick, 2);
    intakeAndIndex.whenHeld(new ParallelCommandGroup(new SetIndexerPower(-1), new SetIntakePower(1)));

    JoystickButton intake = new JoystickButton(mechanismJoystick, 5);
    intake.whenHeld(new SetIntakePower(1));

    JoystickButton driverIntake = new JoystickButton(rightJoystick, 1);
    driverIntake.whenHeld(new SetIntakePower(1));

    JoystickButton outtake = new JoystickButton(mechanismJoystick, 3);
    outtake.whenHeld(new SetIntakePower(-1));

    JoystickButton forwardIndex = new JoystickButton(mechanismJoystick, 11);
    forwardIndex.whenHeld(new SetIndexerPower(-1));

    JoystickButton reverseIndex = new JoystickButton(mechanismJoystick, 12);
    reverseIndex.whenHeld(new ParallelCommandGroup(new SetIndexerPower(1), new SetLoaderPower(-1)));
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
