package frc.robot.subsystems;

// Turret - Part of FRC 6419's 2020 codebase
// Generated 20200204:0902 [2/4/2020:09:02]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.RobotContainer;
import frc.robot.Utilities;

public class Shooter extends SubsystemBase {

  private CANSparkMax shooter0, shooter1;

  private CANPIDController shooterController;
  private CANEncoder shooterEncoder;

  private TalonSRX traverse;
  private double kP, kI, kD;

  private PIDController traverseController;

  public Shooter() {
        shooter0 = new CANSparkMax(Constants.Turret.SHOOTER0, CANSparkMaxLowLevel.MotorType.kBrushless);
        shooter1 = new CANSparkMax(Constants.Turret.SHOOTER1, CANSparkMaxLowLevel.MotorType.kBrushless);

        shooter0.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooter1.setIdleMode(CANSparkMax.IdleMode.kCoast);

        shooter1.follow(shooter0, true);

        shooter0.setOpenLoopRampRate(0.5);
        shooter0.setClosedLoopRampRate(0.5);

        shooterController = shooter0.getPIDController();
        shooterEncoder = shooter0.getEncoder();

        traverse = new TalonSRX(Constants.Turret.TRAVERSE);

        traverse.setNeutralMode(NeutralMode.Brake);
        traverse.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        traverse.configNominalOutputForward(0.05);
        traverse.configNominalOutputReverse(-0.05);
        traverse.configPeakOutputForward(1);
        traverse.configPeakOutputReverse(-1);
        traverse.setSelectedSensorPosition((int) Constants.Turret.TRAVERSE_SOFT_LIMIT);

        if (Constants.Turret.ENABLE_LIMITS) {
            traverse.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            traverse.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
            traverse.configClearPositionOnLimitF(false, 30);
            traverse.configClearPositionOnLimitR(true, 30);
            traverse.configClearPositionOnQuadIdx(false, 30);
        } else {
            traverse.configForwardSoftLimitThreshold(2 * (int) Constants.Turret.TRAVERSE_SOFT_LIMIT);
            traverse.configReverseSoftLimitThreshold(0);
            traverse.configForwardSoftLimitEnable(true);
            traverse.configReverseSoftLimitEnable(true);
        }

        kP = 0.0;
        kI = 0.0;
        kD = 0.0;

        // x input is from -27 to +27.
        traverseController = new PIDController(kP, kI, kD);
        traverseController.setTolerance(1);
        traverseController.setSetpoint(0);
    }

  @Override
  public void periodic() {
    if (lockAcquired()) {
      setTraversePower(-traverseController.calculate(RobotContainer.limelight.getHorizontalAngle()));
    } else {
      setTraversePower(0);
    }
  }

  public void spoolUp() {
    shooterController.setReference(1, ControlType.kDutyCycle);
  }

  public void spoolDown() {
    shooterController.setReference(0, ControlType.kDutyCycle);
  }

  public boolean readyToFire() {
    return shooterEncoder.getVelocity() >= Constants.Turret.FIRING_SPEED;
  }

  public double getShooter0Temperature() {
    return shooter0.getMotorTemperature();
  }

  public double getShooter1Temperature() {
    return shooter1.getMotorTemperature();
  }

  public double getAngle() {
    return encoder2Angle(traverse.getSelectedSensorPosition());
  }

  public void setTraversePower(double power) {
    traverse.set(ControlMode.PercentOutput, power);
  }

  public void setTraverseTargetAngle(double angle) {
    traverse.set(ControlMode.Position, angle2Encoder(angle));
  }

  public boolean lockAcquired() {
    return RobotContainer.limelight.canSeeTarget();
  }

  public void homeEncoder() {
    traverse.setSelectedSensorPosition((int) Constants.Turret.TRAVERSE_SOFT_LIMIT);
  }

  /** Returns the inverse of angle2Encoder() */
  private double encoder2Angle(int encoder) {
    return Utilities.map(encoder, 0, 2 * Constants.Turret.TRAVERSE_SOFT_LIMIT, -Constants.Turret.TRAVERSE_LIMIT_ANGLE,
        Constants.Turret.TRAVERSE_LIMIT_ANGLE);
  }

  /** Returns an encoder target from a given angle relative to the robot from -90 to 90 */
  private int angle2Encoder(double angle) {
    return (int) Utilities.map(angle, -Constants.Turret.TRAVERSE_LIMIT_ANGLE, Constants.Turret.TRAVERSE_LIMIT_ANGLE, 0,
        2 * Constants.Turret.TRAVERSE_SOFT_LIMIT);
  }

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
    super.initSendable(builder);
  }
}