package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.analog.adis16448.frc.ADIS16448_IMU;
import frc.robot.RobotContainer;

import static frc.robot.RobotContainer.drivetrain;

public class Turn extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private double initAngle;
    private double angle;
    private double desiredAngle;
    
    public Turn(double angle) {
        addRequirements(drivetrain);
        this.angle = angle;
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initAngle = drivetrain.getHeading();
        desiredAngle = initAngle + angle;
        System.out.println(String.format("Turning %.2f degrees (from %.2f to %.2f)", angle, initAngle, desiredAngle));

        drivetrain.setHeadingTarget(desiredAngle);
        drivetrain.setHeadingPidEnabled(true);
        drivetrain.drive(0, 0); // Don't move on init
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Turn Complete.");
        RobotContainer.drivetrain.setHeadingPidEnabled(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.drivetrain.atHeadingTarget();
    }
}
