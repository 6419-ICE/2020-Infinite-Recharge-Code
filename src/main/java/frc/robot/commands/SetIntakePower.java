package frc.robot.commands;

// SetIntakePower - Part of FRC 6419's 2020 codebase
// Generated 20200222:1118 [2/22/2020:11:18]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.intake;

public class SetIntakePower extends CommandBase {

    private double power;

    public SetIntakePower(double power) {
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intake.setIntakePower(power);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}