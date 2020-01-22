package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.Encoder;

public class DriveByEncoder extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private CANEncoder leftEncoder, rightEncoder;
    private double distance;
    private double rotations = .204;
    private double InchesPerRotation = 6 * Math.PI / rotations;
    private DriveTrain drive;
    /**
     * Creates a new DriveByEncoder Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveByEncoder(DriveTrain drive, double d) {
        addRequirements(drive);
        leftEncoder = drive.motorEncoderL1;
        rightEncoder = drive.motorEncoderR1;
        this.drive = drive;
        d = distance;
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.drive(0, 0); // Don't move on init
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        drive.drive(1, 1);
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
