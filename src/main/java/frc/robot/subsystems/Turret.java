package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

/**
 * Turret Subsystem
 */
public class Turret extends SubsystemBase {

    private CANSparkMax shooter0, shooter1;

    private CANPIDController shooterController;
    private CANEncoder shooterEncoder;

    private TalonSRX traverse;
    private double kP, kI, kD;

    private DigitalInput homingSwitch;

    private PIDController traverseController;

    private boolean trackingEnabled;
    private boolean lastTrackingState;
    private double scanTimestamp;

    public Turret() {
        shooter0 = new CANSparkMax(Constants.Turret.SHOOTER0, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooter1 = new CANSparkMax(Constants.Turret.SHOOTER1, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooter0.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooter1.setIdleMode(CANSparkMax.IdleMode.kCoast);

        shooter1.follow(shooter0, true);

        shooter0.setOpenLoopRampRate(0.5);
        shooter0.setClosedLoopRampRate(0);

        shooterController = shooter0.getPIDController();
        shooterEncoder = shooter0.getEncoder();

        shooterController.setP(0.00075);
        shooterController.setI(0.0);
        shooterController.setD(20);
        shooterController.setOutputRange(-1, 1);

        traverse = new TalonSRX(Constants.Turret.TRAVERSE);

        /* Make traverse motor have restricted movements with a home position */
        traverse.setNeutralMode(NeutralMode.Brake);
        traverse.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        traverse.configNominalOutputForward(0.05);
        traverse.configNominalOutputReverse(-0.05);
        traverse.configPeakOutputForward(1);
        traverse.configPeakOutputReverse(-1);
        traverse.setSelectedSensorPosition((int) Constants.Turret.TRAVERSE_SOFT_LIMIT);

        traverse.config_kP(0, 0.12);

        if (Constants.Turret.ENABLE_LIMITS) {
            traverse.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            traverse.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            traverse.configClearPositionOnLimitF(false, 30);
            traverse.configClearPositionOnLimitR(true, 30);
            traverse.configClearPositionOnQuadIdx(false, 30);
        } else {
            traverse.configForwardSoftLimitThreshold(angle2Encoder(60));
            traverse.configReverseSoftLimitThreshold(0);
            traverse.configForwardSoftLimitEnable(true);
            traverse.configReverseSoftLimitEnable(true);
        }

        homingSwitch = new DigitalInput(Constants.Turret.HOMING_SWITCH);

        /* PID Constants */
        kP = 0.04;
        kI = 0.0;
        kD = 120;

        // x input is from -27 to +27.
        traverseController = new PIDController(kP, kI, kD);
        traverseController.setTolerance(0.5);
        traverseController.setSetpoint(0);

        trackingEnabled = true;
        scanTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Continuously uses Limelight to check for a vision target
     * If found, the traverse motor will engage to meet the same positional angle as the target
     */
    @Override
    public void periodic() {
        if (trackingEnabled) {
            boolean lockAcquired = lockAcquired();
            if (lockAcquired && !lastTrackingState) {
                System.out.println("[Turret] Target Lock!");
            } else if (!lockAcquired && lastTrackingState) {
                System.out.println("[Turret] Target Lost!");
                scanTimestamp = Timer.getFPGATimestamp();
            }
            lastTrackingState = lockAcquired;
            if (lockAcquired()) {
                setTraversePower(-traverseController.calculate(RobotContainer.limelight.getHorizontalAngle()));
            } else {
                if (DriverStation.getInstance().isAutonomous()) {
                    traverse.set(ControlMode.Position, angle2Encoder(0));
                } else {
                    double timestamp = Timer.getFPGATimestamp() - scanTimestamp;
                    double angleTarget = Utilities.map(
                            Math.sin((Math.PI * timestamp) / 2),
                            -1,
                            1,
                            -87,
                            60);
                    SmartDashboard.putNumber("Turret Target", angleTarget);
                    traverse.set(ControlMode.Position, angle2Encoder(angleTarget));
                }
            }
        }
    }

    /**
     * Engages the turret spool
     */
    public void spoolUp() {
        shooterController.setReference(5000, ControlType.kVelocity);
    }

    public void startSpoolSequence() {
        shooterController.setReference(1, ControlType.kDutyCycle);
    }

    /**
     * Disengages the turret spool
     */
    public void spoolDown() {
        shooterController.setReference(0, ControlType.kDutyCycle);
    }

    public void eject() {
        shooterController.setReference(-0.1, ControlType.kDutyCycle);
    }

    /**
     * @return if the turret has reach max spool speed
     */
    public boolean readyToFire() {
        return shooterEncoder.getVelocity() >= Constants.Turret.FIRING_SPEED;
    }

    /** Overload to add a delay before the turret fires
     * @return if the turret  has reached max spool speed and the delay has passed
     */
    public boolean readyToFire(double init, double wait) {
        double currentTime = Timer.getFPGATimestamp();
        return (currentTime > init + wait) && shooterEncoder.getVelocity() >= Constants.Turret.FIRING_SPEED;
    }

    /**
     * @return spool motor 0's temperature
     */
    public double getShooter0Temperature() {
        return shooter0.getMotorTemperature();
    }

    /**
     * @return spool motor 1's temperature
     */
    public double getShooter1Temperature() {
        return shooter1.getMotorTemperature();
    }

    public double getShooterSpeed() {
        return shooterEncoder.getVelocity();
    }

    /**
     * @return the current angle of the turret
     */
    public double getAngle() {
        return encoder2Angle(traverse.getSelectedSensorPosition());
    }

    /**
     * Set the power of the traverse motor
     *
     * @param power
     */
    public void setTraversePower(double power) {
        if (isHomingSwitchPressed()) {
            traverse.set(ControlMode.PercentOutput, Math.min(0, power));
        } else {
            traverse.set(ControlMode.PercentOutput, power);
        }
    }

    /**
     * Set the target angle for the traverse to "drive" to
     * This feature uses encoders to track the current angle
     *
     * @param angle
     */
    public void setTraverseTargetAngle(double angle) {
        traverse.set(ControlMode.Position, angle2Encoder(angle));
    }

    /**
     * @return if the Limelight has found a vision target
     */
    public boolean lockAcquired() {
        return RobotContainer.limelight.canSeeTarget() && RobotContainer.limelight.getTargetArea() > 0.008;
    }

    public boolean onTarget() {
        return lockAcquired() && Math.abs(RobotContainer.limelight.getHorizontalAngle()) < 1.5;
    }

    /**
     * Set the traverse's current position as the "home" or central position
     */
    public void homeEncoder() {
        traverse.setSelectedSensorPosition((int) Constants.Turret.TRAVERSE_SOFT_LIMIT);
    }

    public void setEncoderPosition(int position) {
        traverse.setSelectedSensorPosition(position);
    }

    /**
     * @return the inverse of angle2Encoder()
     */
    private double encoder2Angle(int encoder) {
        return Utilities.map(encoder, 0, 2 * Constants.Turret.TRAVERSE_SOFT_LIMIT, -Constants.Turret.TRAVERSE_LIMIT_ANGLE,
                Constants.Turret.TRAVERSE_LIMIT_ANGLE);
    }

    /**
     * @return an encoder target from a given angle relative to the robot from -90 to 90
     */
    private int angle2Encoder(double angle) {
        return (int) Utilities.map(angle, -Constants.Turret.TRAVERSE_LIMIT_ANGLE, Constants.Turret.TRAVERSE_LIMIT_ANGLE, 0,
                2 * Constants.Turret.TRAVERSE_SOFT_LIMIT);
    }

    public void setTrackingEnabled(boolean enabled) {
        trackingEnabled = enabled;
    }

    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    public boolean isHomingSwitchPressed() {
        return homingSwitch.get();
    }

    public void disableLimits() {
        traverse.configForwardSoftLimitEnable(false);
        traverse.configReverseSoftLimitEnable(false);
    }

    public void enableLimits() {
        traverse.configForwardSoftLimitEnable(true);
        traverse.configReverseSoftLimitEnable(true);
    }

    /**
     * Build the Shuffleboard Choosers
     *
     * @param builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        setName("Turret");
        builder.addDoubleProperty("Angle", this::getAngle, null);
        builder.addDoubleProperty("Encoder Position", traverse::getSelectedSensorPosition, null);
        builder.addBooleanProperty("Forward Limit Reached", () -> traverse.isFwdLimitSwitchClosed() == 0, null);
        builder.addBooleanProperty("Reverse Limit Reached", () -> traverse.isRevLimitSwitchClosed() == 0, null);
        builder.addBooleanProperty("Ready to Fire", this::readyToFire, null);
        builder.addDoubleProperty("Shooter Speed", shooterEncoder::getVelocity, null);
        builder.addBooleanProperty("Target Lock", this::lockAcquired, null);
        builder.addBooleanProperty("On Target", this::onTarget, null);
        builder.addBooleanProperty("Homing Switch", this::isHomingSwitchPressed, null);
        builder.addBooleanProperty("Tracking Enabled", this::isTrackingEnabled, this::setTrackingEnabled);
        super.initSendable(builder);
    }
}