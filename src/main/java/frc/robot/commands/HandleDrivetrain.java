package frc.robot.commands;

// HandleDrivetrain - Part of FRC 6419's 2020 codebase
// Generated 20200110:1251 [1/10/2020:12:51]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;

import static frc.robot.RobotContainer.drivetrain;

public class HandleDrivetrain extends CommandBase {

    public HandleDrivetrain() {
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.drive(0, 0);
        drivetrain.setHeadingPidEnabled(false);
    }

    @Override
    public void execute() {
        double driveInput = RobotContainer.getRightJoystick().getY();
        double absDriveInput = Math.abs(driveInput);
        double drivePower;
        if (absDriveInput < 0.1) {
            drivePower = 0;
        } else if (absDriveInput < 0.4) {
            drivePower = Utilities.map(absDriveInput, 0.1, 0.4, 0, 0.3);
        } else if (absDriveInput < 0.6) {
            drivePower = Utilities.map(absDriveInput, 0.4, 0.6, 0.3, 0.7);
        } else {
            drivePower = Utilities.map(absDriveInput, 0.6, 1, 0.7, 1);
        }
        drivePower = Math.copySign(drivePower, driveInput);

        double turnInput = -RobotContainer.getLeftJoystick().getX();
        if (Math.abs(turnInput) < 0.1) {
            turnInput = 0;
        } else {
            turnInput = 0.9 * Math.copySign(Math.pow(Math.abs(turnInput), 2), turnInput);
        }

        SmartDashboard.putNumber("Drive", drivePower);
        SmartDashboard.putNumber("Turn", turnInput);

        drivetrain.drive(drivePower + turnInput, drivePower - turnInput);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            drivetrain.drive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}