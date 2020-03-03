package frc.robot.commands;

// SetIndexerPower - Part of FRC 6419's 2020 codebase
// Generated 20200222:1120 [2/22/2020:11:20]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.indexer;

public class SetIndexerPower extends CommandBase {

    private double power;

    public SetIndexerPower(double power) {
        this.power = power;
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        indexer.setPower(power);
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