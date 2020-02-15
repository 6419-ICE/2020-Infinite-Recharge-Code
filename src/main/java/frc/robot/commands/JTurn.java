package frc.robot.commands;

// JTurn - Part of FRC 6419's 2020 codebase
// Generated 20200125:1737 [1/25/2020:17:37]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;

import static frc.robot.RobotContainer.drivetrain;

public class JTurn extends CommandBase {

    private static final double SPEED_FACTOR = 0.5;

    private double angle, entryDirection, switchHeading, stopHeading;

    public JTurn(double angle) {
        addRequirements(drivetrain);
        this.angle = angle;
    }

    @Override
    public void initialize() {
        double startHeading = drivetrain.getHeading();
        switchHeading = startHeading + 0.5 * angle;
        stopHeading = startHeading + angle;
        /*if (Math.abs(RobotContainer.getRightJoystick().getY()) < 0.05) {
            entryDirection = -1;
        } else {
            entryDirection = Math.copySign(1.0, RobotContainer.getLeftJoystick().getY());
        }*/
        // Functionally equivalent as the above code
        if (RobotContainer.getRightJoystick().getY() > 0.05) {
            entryDirection = 1;
            //angle = 160;
        } else {
            entryDirection = -1;
            //angle = 140;
        }
    }

    @Override
    public void execute() {
        double throttle = entryDirection;
        double turn = 1;
        if (drivetrain.getHeading() > switchHeading) {
            throttle *= -1;
        }
        drivetrain.drive(SPEED_FACTOR * (throttle - turn), SPEED_FACTOR * (throttle + turn));
    }

    @Override
    public void end(boolean interrupted) {
        //drivetrain.drive(-entryDirection, -entryDirection);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getHeading() > stopHeading;
    }
}