package frc.robot.commands;

// HomeTurret - Part of FRC 6419's 2020 codebase
// Generated 20200206:1757 [2/6/2020:17:57]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.turret;

public class HomeTurret extends CommandBase {

    public HomeTurret() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.homeEncoder();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}