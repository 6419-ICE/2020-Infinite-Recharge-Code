package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

import static frc.robot.RobotContainer.drivetrain;

/** Turn the robot autonomously using an onboard gyro and accelerometer */
public class Turn extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private double initAngle;
    private double angle;
    private double desiredAngle;
    
    /** Set the desired angle
     * @param angle
     */
    public Turn(double angle) {
        addRequirements(drivetrain);
        this.angle = angle;
      }

    /** Set the heading and initialize the gyro's position */
    @Override
    public void initialize() {
        drivetrain.zeroHeading();
        System.out.println("Turn initialized.");
        //drivetrain.resetHeading();
        initAngle = drivetrain.getGyroHeading();
        desiredAngle = initAngle + angle;
        System.out.println(String.format("Turning %.2f degrees (from %.2f to %.2f)", angle, initAngle, desiredAngle));
        // Turn to the angle
        drivetrain.setMaxMotorSpeed(.5);
        drivetrain.setHeadingTarget(desiredAngle);
        drivetrain.setHeadingPidEnabled(true);
        drivetrain.stop(); // Don't move on init
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drivetrain.drive(-drivetrain.headingOutput(drivetrain.getGyroHeading()), drivetrain.headingOutput(drivetrain.getGyroHeading()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Turn Complete.");
        drivetrain.setHeadingPidEnabled(false);
    }

    // Stop when the angle is reached
    @Override
    public boolean isFinished() {
        return drivetrain.atHeadingTarget();
    }
}
