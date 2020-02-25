package frc.robot.commands;

// SetLoaderPower - Part of FRC 6419's 2020 codebase
// Generated 20200222:1318 [2/22/2020:13:18]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.loader;

public class SetLoaderPower extends CommandBase {

    private double power;

    public SetLoaderPower(double power) {
        this.power = power;
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.setPower(power);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}