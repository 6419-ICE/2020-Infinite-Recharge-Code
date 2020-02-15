/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public static final DriveTrain drivetrain = new DriveTrain();
  public static final Shooter shooter = new Shooter();
  public static final AnalogInput ultrasonic = new AnalogInput(0);
  public static final Limelight limelight = new Limelight();
  // public static final DigitalInput hallEffect = new DigitalInput(1);

  private static Joystick leftJoystick;
  private static Joystick rightJoystick;

  /* Required selections for the SendableChooser */
  public enum autoSelections {
    AUTO_1, AUTO_2, AUTO_3;
  }

  private static SendableChooser<autoSelections> aChooser;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(new HandleDriveTrain());
    shooter.setDefaultCommand(new ShooterDefault());

    /* Multiple Autonomous Selections */
    aChooser = new SendableChooser<>();
    aChooser.setDefaultOption("Default Auto", autoSelections.AUTO_1);
    aChooser.addOption("Auto 1", autoSelections.AUTO_1);
    aChooser.addOption("Auto 2", autoSelections.AUTO_2);
    aChooser.addOption("Auto 3", autoSelections.AUTO_3);

    configureButtonBindings();

    SmartDashboard.putData("Auto Selector", aChooser);
    SmartDashboard.putData(new syncPID());
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or none of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    leftJoystick = new Joystick(Constants.joy1);
    rightJoystick = new Joystick(Constants.joy2);

    JoystickButton shooter = new JoystickButton(rightJoystick, 1); // 1 is always trigger
    shooter.whenHeld(new HandleShooter());
  }

  public static Joystick getLeftJoy() {
    return leftJoystick;
  }

  public static Joystick getRightJoy() {
    return rightJoystick;
  }

  public RobotContainer.autoSelections getSelectedAuto(){
    return aChooser.getSelected();
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  public CommandBase getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
  */
}
