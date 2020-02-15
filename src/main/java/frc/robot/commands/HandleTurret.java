package frc.robot.commands;

// HandleTurret - Part of FRC 6419's 2020 codebase
// Generated 20200204:1733 [2/4/2020:17:33]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.turret;

public class HandleTurret extends CommandBase {

    public HandleTurret() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        //turret.setTraversePower(0);
        turret.spoolDown();
    }

    @Override
    public void execute() {
        if (RobotContainer.getLeftJoystick().getTrigger()) {
            turret.spoolUp();
        } else {
            turret.spoolDown();
        }

        /*if (RobotContainer.getRightJoystick().getPOV() == 90) {
            turret.setTraversePower(1);
        } else if (RobotContainer.getRightJoystick().getPOV() == 270) {
            turret.setTraversePower(-1);
        } else {
            turret.setTraversePower(0);
        }*/
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}