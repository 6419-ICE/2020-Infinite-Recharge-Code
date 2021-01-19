package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class EjectBallFromShooter extends CommandBase {

    public EjectBallFromShooter() {
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void initialize() {
        RobotContainer.shooter.eject();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.shooter.spoolDown();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}