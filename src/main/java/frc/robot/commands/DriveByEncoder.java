package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import com.revrobotics.CANEncoder;

public class DriveByEncoder extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private CANEncoder leftEncoder, rightEncoder;
    private double distance;
    private double rotations = .204;
    private double InchesPerRotation = 6 * Math.PI / rotations;
    private double angle;

    /**
     * Creates a new DriveByEncoder Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveByEncoder(double d) {
        addRequirements(RobotContainer.drivetrain);
        leftEncoder = RobotContainer.drivetrain.motorEncoderL1;
        rightEncoder = RobotContainer.drivetrain.motorEncoderR1;
        distance = d;
        angle = RobotContainer.drivetrain.imu.getAngle();
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // RobotContainer.drivetrain.drive(1, 1);
        super.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }


}
