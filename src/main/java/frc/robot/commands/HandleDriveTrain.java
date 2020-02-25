package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

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
        double power = Utilities.applyDeadband(RobotContainer.getRightJoy().getY(), 0.03);
        double turn = Utilities.applyDeadband(RobotContainer.getLeftJoy().getX(), 0.03);
        power = Math.copySign(Math.abs(Math.pow(power, 2)), power);
        turn = Math.copySign(Math.abs(Math.pow(turn, 2)), turn);
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
