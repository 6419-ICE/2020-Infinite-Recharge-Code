/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
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
  private static Joystick leftJoystick;
  private static Joystick rightJoystick;
  public final CommandGroupBase m_AutoGroup = new AutoGroup();
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand(new HandleDriveTrain(drivetrain));
    shooter.setDefaultCommand(new HandleShooter(shooter));
    configureButtonBindings();

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
  }

  public static Joystick getLeftJoy() {
    return leftJoystick;
  }

  public static Joystick getRightJoy() {
    return rightJoystick;
  }

  public CommandGroupBase getAutonomousCommand(){
    return m_AutoGroup;
  }

  public static boolean getShooterButton(){
    return rightJoystick.getRawButton(1);
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
