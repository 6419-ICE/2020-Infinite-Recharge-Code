package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;


public class DriveByEncoder extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private double leftStartValue, rightStartValue;
    private CANEncoder leftEncoder, rightEncoder;
    private double distance;
    private boolean leftFinished, rightFinished;
    private ADIS16448_IMU imu;
    private double originalHeader;

    /**
     * Creates a new DriveByEncoder Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveByEncoder(DriveTrain drive, double distance) {
        addRequirements(drive);
        leftEncoder = drive.motorEncoderL1;
        rightEncoder = drive.motorEncoderR1;
        imu = drive.imu;
        this.distance = distance;
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
        originalHeader = imu.getAngle();
        resetEncoders();
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        leftFinished = leftEncoder.getPosition() - leftStartValue >= distance;
        rightFinished = leftEncoder.getPosition() - rightStartValue >= distance;

        if(imu.getAngle() <= originalHeader - 2 && imu.getAngle() >= originalHeader + 2){
            if(!leftFinished) {
                RobotContainer.drivetrain.drive(0.3, 0);
            }
    
            if(!rightFinished) {
                RobotContainer.drivetrain.drive(0, 0.3);
            }
        } 
        else if(imu.getAngle() <= originalHeader - 2) {
            RobotContainer.drivetrain.drive(0.3, -0.3);
        }else if(imu.getAngle() >= originalHeader + 2) {
            RobotContainer.drivetrain.drive(-0.3, 0.3);
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
