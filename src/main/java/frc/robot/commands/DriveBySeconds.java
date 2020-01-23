package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

public class DriveBySeconds extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Timer timer;
    private double timeLimit;
    /**
     * Creates a new HandleDriveTrain Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveBySeconds(double seconds) {
        timeLimit = seconds;
        timer = new Timer();
        addRequirements(RobotContainer.drivetrain);
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        RobotContainer.drivetrain.drive(0, 0); // Don't move on init
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() > timeLimit) {
            RobotContainer.drivetrain.drive(0.0, 0.0);
        } else {
            RobotContainer.drivetrain.drive(0.3, 0.3);
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
        return false;
    }
}
