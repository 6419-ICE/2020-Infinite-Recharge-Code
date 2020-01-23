package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.analog.adis16448.frc.ADIS16448_IMU;
import frc.robot.RobotContainer;

public class Turn extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private boolean done;
    private ADIS16448_IMU imu;
    private double initAngle;
    private double angle;
    private double desiredAngle;
    
    public Turn(double angle) {
        imu = RobotContainer.drivetrain.imu;
        addRequirements(RobotContainer.drivetrain);
        this.angle = angle;
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
        imu.reset();
        initAngle = imu.getAngle();
        System.out.println(initAngle);
        desiredAngle = initAngle + angle;
        System.out.println(desiredAngle);
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        if (Math.abs(imu.getAngle()) > Math.abs(desiredAngle)) {
            RobotContainer.drivetrain.drive(0.0, 0.0);
            System.out.println(imu.getAngle());
            done = true;
        } else {
            RobotContainer.drivetrain.drive(.2, -.2);
        }
        super.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}
