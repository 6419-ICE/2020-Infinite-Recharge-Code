package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** Handle the Drivetrain in Teleop */
public class HandleDriveTrain extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    /**
     * Creates a new HandleDriveTrain Command.
     *
     */
    public HandleDriveTrain() {
        addRequirements(RobotContainer.drivetrain);
      }

    // When in doubt, stop the motors
    @Override
    public void initialize() {
        RobotContainer.drivetrain.stop(); // Don't move on init
    }

    // Used for Teleop control and displaying power values
    @Override
    public void execute() {
        double power = RobotContainer.getRightJoy().getY();
        double turn = RobotContainer.getLeftJoy().getX();
        power = Math.copySign(Math.abs(Math.pow(power, 2)), power);
        SmartDashboard.putNumber("Power", power);
        SmartDashboard.putNumber("Turn", turn);

        RobotContainer.drivetrain.arcadeDrive(power, turn);
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
