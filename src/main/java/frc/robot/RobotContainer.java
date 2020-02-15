/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.HandleTurret;
import frc.robot.commands.HomeTurret;
import frc.robot.commands.JTurn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public static Drivetrain drivetrain = new Drivetrain();
    public static Limelight limelight = new Limelight();
    public static Turret turret = new Turret();

    private SendableChooser<CommandBase> autonomousChooser;

    private static Joystick leftJoystick, rightJoystick;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain.initDefaultCommand();

        turret.setDefaultCommand(new HandleTurret());

        SmartDashboard.putData(drivetrain);
        SmartDashboard.putData(turret);

        SmartDashboard.putData(new HomeTurret());

        autonomousChooser = new SendableChooser<>();
        autonomousChooser.setDefaultOption("None", null);

        // Configure the button bindings
        configureOperatorInterface();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureOperatorInterface() {
        leftJoystick = new Joystick(0);
        rightJoystick = new Joystick(1);

        leftJoystick.setXChannel(0);
        leftJoystick.setYChannel(1);
        leftJoystick.setTwistChannel(2);
        leftJoystick.setThrottleChannel(3);

        rightJoystick.setXChannel(0);
        rightJoystick.setYChannel(1);
        rightJoystick.setTwistChannel(2);
        rightJoystick.setThrottleChannel(3);

        JoystickButton jTurnButton = new JoystickButton(rightJoystick, 1);
        jTurnButton.whenPressed(new JTurn(180));
    }

    public static Joystick getLeftJoystick() {
      return leftJoystick;
    }

    public static Joystick getRightJoystick() {
      return rightJoystick;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autonomousChooser.getSelected();
    }
}
