package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class HandleDriveTrain extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new HandleDriveTrain Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public HandleDriveTrain(DriveTrain drive) {
        addRequirements(drive);
      }

        // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.drivetrain.stop(); // Don't move on init
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // RobotContainer.drivetrain.drive(RobotContainer.getLeftJoy().getRawAxis(1), RobotContainer.getRightJoy().getRawAxis(1));
        RobotContainer.drivetrain.arcadeDrive(RobotContainer.getLeftJoy().getY(), RobotContainer.getRightJoy().getX());
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
