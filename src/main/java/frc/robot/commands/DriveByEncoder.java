package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class DriveByEncoder extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private double leftStartValue, rightStartValue;
    private CANEncoder leftEncoder, rightEncoder;
    private double distance;
    private boolean leftFinished, rightFinished;

    /**
     * Creates a new DriveByEncoder Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveByEncoder(DriveTrain drive, double distance) {
        addRequirements(drive);
        leftEncoder = drive.motorEncoderL1;
        rightEncoder = drive.motorEncoderR1;
        this.distance = distance;
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
        resetEncoders();
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        leftFinished = leftEncoder.getPosition() - leftStartValue >= distance;
        rightFinished = leftEncoder.getPosition() - rightStartValue >= distance;

        if(!leftFinished) {
            RobotContainer.drivetrain.drive(0.3, 0);
        }

        if(!rightFinished) {
            RobotContainer.drivetrain.drive(0, 0.3);
        }

        super.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(final boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return leftFinished && rightFinished;
    }

    private void resetEncoders() {
        leftStartValue = leftEncoder.getPosition();
        rightStartValue = rightEncoder.getPosition();
    }
}
