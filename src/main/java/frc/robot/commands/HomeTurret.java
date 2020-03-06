package frc.robot.commands;

// HomeTurret - Part of FRC 6419's 2020 codebase
// Generated 20200221:2038 [2/21/2020:20:38]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.shooter;

public class HomeTurret extends CommandBase {

    public HomeTurret() {
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTrackingEnabled(false);
        shooter.disableLimits();
        shooter.setTraversePower(0.1);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTraversePower(0);
        shooter.enableLimits();
        if (!interrupted) {
            shooter.setEncoderPosition(2 * (int) Constants.Turret.TRAVERSE_SOFT_LIMIT);
        }
        shooter.setTrackingEnabled(true);
        System.out.println("Homing Complete.");
    }

    @Override
    public boolean isFinished() {
        return shooter.isHomingSwitchPressed();
    }
}