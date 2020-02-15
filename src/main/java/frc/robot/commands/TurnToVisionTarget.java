package frc.robot.commands;

// TurnToVisionTarget - Part of FRC 6419's 2020 codebase
// Generated 20200110:1325 [1/10/2020:13:25]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.*;
import static frc.robot.RobotContainer.drivetrain;
import static frc.robot.RobotContainer.limelight;

public class TurnToVisionTarget extends CommandBase {

    private PIDController controller;

    public TurnToVisionTarget() {
        addRequirements(drivetrain, limelight);

        Preferences prefs = Preferences.getInstance();
        boolean prefsFound = true;

        if (!prefs.containsKey(VISION_KP_TAG)) {
            prefsFound = false;
            prefs.putDouble(VISION_KP_TAG, 1);
        }
        if (!prefs.containsKey(VISION_KI_TAG)) {
            prefsFound = false;
            prefs.putDouble(VISION_KI_TAG, 0);
        }
        if (!prefs.containsKey(VISION_KD_TAG)) {
            prefsFound = false;
            prefs.putDouble(VISION_KD_TAG, 1);
        }

        if (!prefsFound) {
            DriverStation.reportWarning("One or more preference values are missing. Default values have been used.", true);
        }

        controller = new PIDController(
                prefs.getDouble(VISION_KP_TAG, 1),
                prefs.getDouble(VISION_KI_TAG, 0),
                prefs.getDouble(VISION_KD_TAG, 1)
        );
        controller.setTolerance(1);
        controller.setSetpoint(0);
    }

    @Override
    public void initialize() {
        drivetrain.drive(0, 0);
    }

    @Override
    public void execute() {
        double pidOutput = controller.calculate(limelight.getHorizontalAngle());
        drivetrain.drive(pidOutput, -pidOutput);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return !limelight.canSeeTarget() || controller.atSetpoint();
    }
}