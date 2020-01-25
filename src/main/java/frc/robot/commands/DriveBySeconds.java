package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class DriveBySeconds extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Timer timer;
    private double timeLimit;
    private boolean done;
    private boolean backwards;

    public DriveBySeconds(double seconds, boolean backwards) {
        timeLimit = seconds;
        this.backwards = backwards;
        timer = new Timer();
        addRequirements(RobotContainer.drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        done = false;
        timer.reset();
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() > timeLimit) {
            RobotContainer.drivetrain.drive(0.0, 0.0);
            
            done = true;
        } else {
            if (backwards) {
                RobotContainer.drivetrain.drive(-0.25, -0.25);
            } else {
                RobotContainer.drivetrain.drive(0.25, 0.25);
            }
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
