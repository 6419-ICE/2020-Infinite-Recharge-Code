package frc.robot.subsystems;

// Drivetrain - Part of FRC 6419's 2020 codebase
// Generated 20200110:1000 [1/10/2020:10:00]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.commands.HandleDrivetrain;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {

    private ADIS16448_IMU imu;

    private TalonFX left0, left1, left2,
                        right0, right1, right2;

    private PIDController headingPidController;

    private boolean headingPidEnabled;

    private double left0MaxCurrent,
                   left1MaxCurrent,
                   left2MaxCurrent,
                   right0MaxCurrent,
                   right1MaxCurrent,
                   right2MaxCurrent;

    public Drivetrain() {
        imu = new ADIS16448_IMU();
        imu.calibrate();

        left0 = new TalonFX(Constants.Drivetrain.LEFT0);
        left1 = new TalonFX(Constants.Drivetrain.LEFT1);
        left2 = new TalonFX(Constants.Drivetrain.LEFT2);

        right0 = new TalonFX(Constants.Drivetrain.RIGHT0);
        right1 = new TalonFX(Constants.Drivetrain.RIGHT1);
        right2 = new TalonFX(Constants.Drivetrain.RIGHT2);

        left0.setNeutralMode(NeutralMode.Brake);
        left1.setNeutralMode(NeutralMode.Brake);
        left2.setNeutralMode(NeutralMode.Brake);

        right0.setNeutralMode(NeutralMode.Brake);
        right1.setNeutralMode(NeutralMode.Brake);
        right2.setNeutralMode(NeutralMode.Brake);

        Preferences prefs = Preferences.getInstance();
        boolean prefsFound = true;

        if (!prefs.containsKey(KP_TAG)) {
            prefsFound = false;
            prefs.putDouble(KP_TAG, 1);
        }
        if (!prefs.containsKey(KI_TAG)) {
            prefsFound = false;
            prefs.putDouble(KI_TAG, 0);
        }
        if (!prefs.containsKey(KD_TAG)) {
            prefsFound = false;
            prefs.putDouble(KD_TAG, 1);
        }

        if (!prefsFound) {
            DriverStation.reportWarning("One or more preference values are missing. Default values have been used.", true);
        }

        headingPidController = new PIDController(
                prefs.getDouble(KP_TAG, 1),
                prefs.getDouble(KI_TAG, 0),
                prefs.getDouble(KD_TAG, 1)
        );

        headingPidController.setTolerance(0.01);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new HandleDrivetrain());
    }

    @Override
    public void periodic() {
        if (headingPidEnabled) {
            double pidOutput = headingPidController.calculate(getHeading());
            drive(-pidOutput, pidOutput);
        }
        if (left0.getStatorCurrent() > left0MaxCurrent) {
            left0MaxCurrent = left0.getStatorCurrent();
        }
        if (left1.getStatorCurrent() > left1MaxCurrent) {
            left1MaxCurrent = left1.getStatorCurrent();
        }
        if (left2.getStatorCurrent() > left2MaxCurrent) {
            left2MaxCurrent = left2.getStatorCurrent();
        }
        if (right0.getStatorCurrent() > right0MaxCurrent) {
            right0MaxCurrent = right0.getStatorCurrent();
        }
        if (right1.getStatorCurrent() > right1MaxCurrent) {
            right1MaxCurrent = right1.getStatorCurrent();
        }
        if (right2.getStatorCurrent() > right2MaxCurrent) {
            right2MaxCurrent = right2.getStatorCurrent();
        }
    }

    public void drive(double left, double right) {
        left0.set(TalonFXControlMode.PercentOutput, -left);
        left1.set(TalonFXControlMode.PercentOutput, -left);
        left2.set(TalonFXControlMode.PercentOutput, -left);
        right0.set(TalonFXControlMode.PercentOutput, right);
        right1.set(TalonFXControlMode.PercentOutput, right);
        right2.set(TalonFXControlMode.PercentOutput, right);
    }

    public void setHeadingPidEnabled(boolean enabled) {
        headingPidEnabled = enabled;
    }

    public boolean isHeadingPidEnabled() {
        return headingPidEnabled;
    }

    public void setHeadingSetpoint(double setpoint) {
        headingPidController.setSetpoint(setpoint);
    }

    public double getHeadingSetpoint() {
        return headingPidController.getSetpoint();
    }

    public boolean atHeadingSetpoint() {
        return headingPidController.atSetpoint();
    }

    public double getHeading() {
        return imu.getAngleZ();
    }

    public void resetHeading() {
        imu.reset();
    }

    public void resetMaxCurrent() {
        left0MaxCurrent
                = left1MaxCurrent
                = left2MaxCurrent
                = right0MaxCurrent
                = right1MaxCurrent
                = right2MaxCurrent
                = 0;
    }

    public double getLeft0MaxCurrent() {
        return left0MaxCurrent;
    }

    public double getLeft1MaxCurrent() {
        return left1MaxCurrent;
    }

    public double getLeft2MaxCurrent() {
        return left2MaxCurrent;
    }

    public double getRight0MaxCurrent() {
        return right0MaxCurrent;
    }

    public double getRight1MaxCurrent() {
        return right1MaxCurrent;
    }

    public double getRight2MaxCurrent() {
        return right2MaxCurrent;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        setName("Drivetrain");
        builder.addDoubleProperty("Left 0 Temperature", left0::getTemperature, null);
        builder.addDoubleProperty("Left 1 Temperature", left1::getTemperature, null);
        builder.addDoubleProperty("Left 2 Temperature", left2::getTemperature, null);
        builder.addDoubleProperty("Right 0 Temperature", right0::getTemperature, null);
        builder.addDoubleProperty("Right 1 Temperature", right1::getTemperature, null);
        builder.addDoubleProperty("Right 2 Temperature", right2::getTemperature, null);
        builder.addDoubleProperty("Heading", this::getHeading, null);

        builder.addDoubleProperty("Left 0 Max Current", this::getLeft0MaxCurrent, null);
        builder.addDoubleProperty("Left 1 Max Current", this::getLeft1MaxCurrent, null);
        builder.addDoubleProperty("Left 2 Max Current", this::getLeft2MaxCurrent, null);
        builder.addDoubleProperty("Right 0 Max Current", this::getRight0MaxCurrent, null);
        builder.addDoubleProperty("Right 1 Max Current", this::getRight1MaxCurrent, null);
        builder.addDoubleProperty("Right 2 Max Current", this::getRight2MaxCurrent, null);

        builder.addDoubleProperty("Left 0 Current", left0::getStatorCurrent, null);
        builder.addDoubleProperty("Left 1 Current", left1::getStatorCurrent, null);
        builder.addDoubleProperty("Left 2 Current", left2::getStatorCurrent, null);
        builder.addDoubleProperty("Right 0 Current", right0::getStatorCurrent, null);
        builder.addDoubleProperty("Right 1 Current", right1::getStatorCurrent, null);
        builder.addDoubleProperty("Right 2 Current", right2::getStatorCurrent, null);
        super.initSendable(builder);
    }
}