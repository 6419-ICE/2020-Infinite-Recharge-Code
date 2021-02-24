/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  public static final Hanging hanger = null;
  public static final Compressor compressor = null;

  private static Joystick leftJoystick;
  private static Joystick rightJoystick;
  private static Joystick mechanismJoystick;
  public static JoystickButton hangingButton;

  // Select an autonomous command via Shuffleboard
  private static SendableChooser<CommandBase> autoChooser;

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
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);
    autoChooser.addOption("Path A Red", new PathARed());
    /*
    aChooser.addOption("Trench Run", new TrenchRunAuto());
    aChooser.addOption("Center to Mid", new CenterMidAuto());
    aChooser.addOption("Left to Mid", new LeftMidAuto());
    aChooser.addOption("Test", new Turn(180));
    */

    // Set button binding instances
    configureButtonBindings();

    SmartDashboard.putData("Autonomous", autoChooser);
    /*SmartDashboard.putData(new syncPID());
    SmartDashboard.putData(new HomeTurret());*/
  }

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
    JoystickButton driverIntake = new JoystickButton(rightJoystick, 1);

    Trigger intakeTrigger = new Trigger(() -> intake.get() || driverIntake.get());
    intakeTrigger.whileActiveOnce(new SetIntakePower(-1));

    JoystickButton outtake = new JoystickButton(mechanismJoystick, Constants.outtakeBtn);
    outtake.whenHeld(new SetIntakePower(1));

    JoystickButton forwardIndex = new JoystickButton(mechanismJoystick, Constants.indexForward);
    forwardIndex.whenHeld(new SetIndexerPower(-1));

    JoystickButton reverseIndex = new JoystickButton(mechanismJoystick, Constants.indexReverse);
    reverseIndex.whenHeld(new ParallelCommandGroup(new EjectBallFromShooter(), new SetIndexerPower(1), new SetLoaderPower(-1)));

    JoystickButton centerTurret = new JoystickButton(leftJoystick, 1);
    centerTurret.whenHeld(new CenterTurret());

    hangingButton = new JoystickButton(mechanismJoystick, Constants.liftingButton);
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

  public static JoystickButton getHangingButton(){
    return hangingButton;
  }

  /** Return the selected autonomous command  */
  public CommandBase getSelectedAuto(){
    return autoChooser.getSelected();
  }
  public Command TrajectoryAttempt(){
    var autoVoltageConstant = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts, Constants.Drivetrain.ksVoltsSecondsPerMeter, Constants.Drivetrain.ksVoltsSecondsSquaredPerMeter), Constants.Drivetrain.kDriveKinematics, 7);
    
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drivetrain.kMaxSpeedMetersPerSecond, Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.Drivetrain.kDriveKinematics).addConstraint(autoVoltageConstant);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);
    
    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, drivetrain::getPose, new RamseteController(Constants.Drivetrain.kRamseteB,  Constants.Drivetrain.kRamseteZeta), (new SimpleMotorFeedforward(Constants.Drivetrain.ksVolts, Constants.Drivetrain.ksVoltsSecondsPerMeter, Constants.Drivetrain.ksVoltsSecondsSquaredPerMeter)), Constants.Drivetrain.kDriveKinematics, drivetrain::getWheelSpeeds, new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0), new PIDController(Constants.Drivetrain.kPDriveVel, 0, 0), drivetrain::tankDriveVolts, drivetrain);

    drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }
}

