package frc.robot.commands;

// CenterTurret - Part of FRC 6419's 2020 codebase
// Generated 20200303:1941 [3/3/2020:19:41]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.shooter;

public class AimTurretLeft extends CommandBase {

    public AimTurretLeft() {
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTrackingEnabled(false);
        shooter.setTraverseTargetAngle(-50);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTrackingEnabled(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}