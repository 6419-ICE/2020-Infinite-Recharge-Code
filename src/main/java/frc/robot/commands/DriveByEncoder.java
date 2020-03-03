/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import static frc.robot.RobotContainer.drivetrain;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class DriveByEncoder extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private TalonFX leftEncoder, rightEncoder;

    private double distance;
    private final double inchesPerRotation = Constants.Drivetrain.inchesPerRotation;

    private double leftStart, rightStart;

    /** Get the motors from drivetrain and set the new distance
     * @param d - The desired distance to travel by encoders
     */
    public DriveByEncoder(double d) {
        addRequirements(drivetrain);
        
        leftEncoder = drivetrain.getLeftMotors();
        rightEncoder = drivetrain.getRightMotors();

        distance = d;
    }

    /** Reset the encoders and set the new position to drive to */
    @Override
    public void initialize() {
        System.out.println("Driving");

        leftEncoder.setSelectedSensorPosition(0, 0, 30);
        rightEncoder.setSelectedSensorPosition(0, 0, 30);

        drivetrain.stop(); // Don't move on init
        drivetrain.setSetpoints(distance, distance);
    }

    /** Report to Shuffleboard of the current encoder position */
    @Override
    public void execute() {
        SmartDashboard.putNumber("Left Encoder", Math.abs((leftEncoder.getSelectedSensorPosition()/4096 * 2) * inchesPerRotation));
        SmartDashboard.putNumber("Right Encoder", Math.abs((rightEncoder.getSelectedSensorPosition()/4096 * 2) * inchesPerRotation));

        /*
         * Drive to set number of inches if (leftEncoder.getPosition() *
         * inchesPerRotation < distance && rightEncoder.getPosition() *
         * -inchesPerRotation < distance){ RobotContainer.drivetrain.drive(0.5, 0.5); }
         * else { RobotContainer.drivetrain.drive(0, 0); done = true; }
         */

        /*
         * Stay in a straight line if (imu.getAngle() >= originalHeader + buffer){
         * RobotContainer.drivetrain.drive(0.25, 0.5); } else if (imu.getAngle() <=
         * originalHeader - buffer){ RobotContainer.drivetrain.drive(0.5, 0.25); }
         */

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {
        System.out.println(String.format("Drive Complete: interrupted: %s", interrupted ? "yes" : "no"));
        drivetrain.stop();
    }

    /** @return if the encoders have reached the desired position */
    @Override
    public boolean isFinished() {
        return Math.abs((rightEncoder.getSelectedSensorPosition() / 4096 * 2) * inchesPerRotation) >= distance - 8
                || Math.abs((leftEncoder.getSelectedSensorPosition() / 4096 * 2) * inchesPerRotation) >= distance - 8;
    }
}
