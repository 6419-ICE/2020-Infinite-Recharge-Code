package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import com.revrobotics.CANEncoder;

public class DriveByEncoder extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private CANEncoder leftEncoder, rightEncoder;

    private double distance;
    private final double inchesPerRotation = RobotContainer.drivetrain.getInchesPerRotation();

    public DriveByEncoder(double d) {
        addRequirements(RobotContainer.drivetrain);
        leftEncoder = RobotContainer.drivetrain.motorEncoderL1;
        rightEncoder = RobotContainer.drivetrain.motorEncoderR1;

        distance = d;
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Driving");

        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
        RobotContainer.drivetrain.setSetpoints(distance, distance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    
        /* Drive to set number of inches 
        if (leftEncoder.getPosition() * inchesPerRotation < distance && rightEncoder.getPosition() * -inchesPerRotation < distance){
            RobotContainer.drivetrain.drive(0.5, 0.5);
        } else {
            RobotContainer.drivetrain.drive(0, 0);
            done = true;
        }
        */

        /* Stay in a straight line
        if (imu.getAngle() >= originalHeader + buffer){
            RobotContainer.drivetrain.drive(0.25, 0.5);
        } else if (imu.getAngle() <= originalHeader - buffer){
            RobotContainer.drivetrain.drive(0.5, 0.25);
        }
        */        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return leftEncoder.getPosition() > distance / inchesPerRotation || Math.abs(rightEncoder.getPosition()) > distance / inchesPerRotation;
    }


}
