package frc.robot.commands;

// LoaderDefault - Part of FRC 6419's 2020 codebase
// Generated 20200221:2130 [2/21/2020:21:30]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

import static frc.robot.RobotContainer.loader;

public class LoaderDefault extends CommandBase {

    public LoaderDefault() {
        addRequirements(loader);
    }

    @Override
    public void initialize() {
        loader.stopLoader();
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