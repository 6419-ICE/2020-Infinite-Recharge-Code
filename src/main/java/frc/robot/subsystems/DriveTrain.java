package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.HandleDriveTrain;

public class DriveTrain extends SubsystemBase {

    // The motors on the left side of the drive.
    private final WPI_TalonFX[] m_leftMotors = { new WPI_TalonFX(Constants.FRONT_ONE_PIN),
            new WPI_TalonFX(Constants.FRONT_TWO_PIN), new WPI_TalonFX(Constants.FRONT_THREE_PIN) };
    private final SpeedControllerGroup m_leftControllerGroup = new SpeedControllerGroup(m_leftMotors);

    // The motors on the right side of the drive.
    private final WPI_TalonFX[] m_rightMotors = { new WPI_TalonFX(Constants.BACK_ONE_PIN),
            new WPI_TalonFX(Constants.BACK_TWO_PIN), new WPI_TalonFX(Constants.BACK_THREE_PIN) };
    private final SpeedControllerGroup m_rightControllerGroup = new SpeedControllerGroup(m_rightMotors);

    // The robot's drive
    private final DifferentialDrive m_drive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

    // The left-side and right-side drive encoders
    private final TalonFXSensorCollection m_leftEncoder, m_rightEncoder;

    // The gyro sensor
    private final Gyro m_gyro;
    public ADIS16448_IMU imu = new ADIS16448_IMU();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    private boolean headingPidEnabled;

    // PID Controller
    public static double kP = 0.025, kI = 0, kD = 0.2, kF = 0;
    private PIDController headingPidController;

    /**
     * Creates a new DriveTrain subsystem.
     */
    public DriveTrain() {
        // Sets the feedback sensor for the motors
        m_leftMotors[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_rightMotors[0].configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

        // Sets the encoders
        m_leftEncoder = m_leftMotors[0].getSensorCollection();
        m_rightEncoder = m_rightMotors[0].getSensorCollection();

        resetEncoders();

        /* Reset the Accelerometer/Gyro/etc. */
        imu.setYawAxis(ADIS16448_IMU.IMUAxis.kZ);
        imu.calibrate();
        m_gyro = imu;

        m_odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(-getGyroHeading())));

        m_leftControllerGroup.setInverted(false);
        m_rightControllerGroup.setInverted(false);
        /* Six-motor Falcon 500 Drivetrain */
        TalonFX left1 = new TalonFX(Constants.FRONT_ONE_PIN);
        TalonFX left2 = new TalonFX(Constants.FRONT_TWO_PIN);
        TalonFX left3 = new TalonFX(Constants.FRONT_THREE_PIN);
        TalonFX right1 = new TalonFX(Constants.BACK_ONE_PIN);
        TalonFX right2 = new TalonFX(Constants.BACK_TWO_PIN);
        TalonFX right3 = new TalonFX(Constants.BACK_THREE_PIN);

        left1.setNeutralMode(NeutralMode.Brake);
        left2.setNeutralMode(NeutralMode.Brake);
        left3.setNeutralMode(NeutralMode.Brake);
        right1.setNeutralMode(NeutralMode.Brake);
        right2.setNeutralMode(NeutralMode.Brake);
        right3.setNeutralMode(NeutralMode.Brake);

        left2.follow(left1);
        left3.follow(left1);

        right2.follow(right1);
        right3.follow(right1);

        /* PID Constants */
        kP = 0.025;
        kI = 0;
        kD = 0.2;
        kF = 0;

        left1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);// Try Absolute instead of
                                                                                          // relative
        right1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

        left1.setSensorPhase(false);
        right1.setSensorPhase(false);

        left1.configNominalOutputForward(0, 30);
        left1.configNominalOutputReverse(0, 30);
        left1.configPeakOutputForward(1, 30);
        left1.configPeakOutputReverse(-1, 30);

        right1.configNominalOutputForward(0, 30);
        right1.configNominalOutputReverse(0, 30);
        right1.configPeakOutputForward(1, 30);
        right1.configPeakOutputReverse(-1, 30);

        left1.configAllowableClosedloopError(0, 0, 30);
        right1.configAllowableClosedloopError(0, 0, 30);

        left1.configClosedloopRamp(0.1);
        right1.configClosedloopRamp(0.1);

        /* Config PID values: Config 0 */
        left1.config_kP(0, kP); // 0 is the slot index for this current PID config
        left1.config_kI(0, kI);
        left1.config_kD(0, kD);
        left1.config_kF(0, kF);

        right1.config_kP(0, kP); // 0 is the slot index for this current PID config
        right1.config_kI(0, kI);
        right1.config_kD(0, kD);
        right1.config_kF(0, kF);

        left1.setSelectedSensorPosition(0);
        right1.setSelectedSensorPosition(0);

        /*
         * CHANGES WITH TALONFX:
         * 
         * Gets the position of the encoder: left1.getSelectedSensorPosition();
         * 
         * /* For conversion when using Motion Profiling:
         * leftController.setSmartMotionMaxAccel(3000, 0);
         * leftController.setSmartMotionMaxVelocity(5000, 0);
         * leftController.setSmartMotionMinOutputVelocity(60, 0);
         * 
         * rightController.setSmartMotionMaxAccel(3000, 0);
         * rightController.setSmartMotionMaxVelocity(5000, 0);
         * rightController.setSmartMotionMinOutputVelocity(60, 0);
         */

        /* Change PID in Shuffleboard */
        Preferences prefs = Preferences.getInstance();

        if (!prefs.containsKey("Heading P")) {
            prefs.putDouble("Heading P", 0.045);
        }
        if (!prefs.containsKey("Heading D")) {
            prefs.putDouble("Heading D", 0);
        }
        if (!prefs.containsKey("Turbo Speed")) {
            prefs.putDouble("Turbo Speed", 0.6);
        }

        headingPidController = new PIDController(0.0061, // .00435 for 50%, .0061 for 25%
                0, 0.00001);
        headingPidController.setTolerance(Constants.Drivetrain.headingPidTolerance);
    }

    /** @return left motors for use in Autonomous commands */
    public TalonFX getLeftMotors() {
        return m_leftMotors[0];
    }

    /** @return right motors for use in Autonomous commands */
    public TalonFX getRightMotors() {
        return m_rightMotors[0];
    }

    public void setMaxMotorSpeed(double speed) {
        m_leftMotors[0].configPeakOutputForward(speed);
        m_leftMotors[0].configPeakOutputReverse(-speed);
        m_rightMotors[0].configPeakOutputForward(speed);
        m_rightMotors[0].configPeakOutputReverse(-speed);
    }

    /**
     * Set each set of motors to a given power percentage
     * 
     * @param l - Left Power
     * @param r - Right Power
     */
    public void drive(double l, double r) {
        m_leftControllerGroup.set(l * Constants.Drivetrain.speedLmt);
        m_rightControllerGroup.set(-r * Constants.Drivetrain.speedLmt);
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        m_leftControllerGroup.setVoltage(leftVolts);
        m_rightControllerGroup.setVoltage(-rightVolts);
        m_drive.feed();
    }

    /**
     * CTRE MOTOR ARCADE DRIVE Set each set of motors to a given power percentage
     * based on Joystick axes This is strictly used to set motors during Teleop
     * 
     * @param p - Left joystick input (throttle power)
     * @param t - Right joystick input (turn power)
     */
    /*
     * public void arcadeDrive(double p, double t) { double powValue = p * -1;
     * double turnValue = t ; // Do not need to invert turn input
     * drive(Constants.Drivetrain.speedLmt * (turnValue + powValue),
     * -Constants.Drivetrain.speedLmt * (turnValue - powValue)); }
     */

    /** Stop the motors entirely */
    public void stop() {
        m_leftControllerGroup.set(0.0);
        m_rightControllerGroup.set(0.0);
    }

    public void diffStop() {
        m_drive.arcadeDrive(0, 0);
    }

    /** Refresh PID with Shuffleboard tunings */
    public void syncPIDTunings() {
        Preferences prefs = Preferences.getInstance();
        headingPidController.setPID(prefs.getDouble("Heading P", 0.015), 0, prefs.getDouble("Heading D", 0));
    }

    /**
     * Drive to a set of points by Encoder Drive
     * 
     * @param left  - Left point
     * @param right - Right point
     */

    public void setSetpoints(double left, double right) {
        m_leftMotors[0].set(TalonFXControlMode.Position, left / Constants.Drivetrain.inchesPerRotation / 2 * 4096);
        m_rightMotors[0].set(TalonFXControlMode.Position, -right / Constants.Drivetrain.inchesPerRotation / 2 * 4096);

    }

    /**
     * Specify wether to enable or disable headingPID
     * 
     * @param enabled - Enable or disable
     */
    public void setHeadingPidEnabled(boolean enabled) {
        headingPidEnabled = enabled;
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_gyro.getRotation2d().getDegrees();
    }

    /**
     * Set the target heading to turn to
     * 
     * @param target - The specified target heading
     */
    public void setHeadingTarget(double target) {
        headingPidController.setSetpoint(target);
    }

    /** @return the target heading */
    public double getHeadingTarget() {
        return headingPidController.getSetpoint();
    }

    /** @return if the target heading has been reached */
    public boolean atHeadingTarget() {
        return headingPidController.atSetpoint();
    }

    public double getGyroHeading() {
        return (m_gyro.getAngle()*2);
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -m_gyro.getRate();
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public TalonFXSensorCollection getLeftEncoder() {
        return m_leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public TalonFXSensorCollection getRightEncoder() {
        return m_rightEncoder;
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        imu.reset();
        m_gyro.reset();
    }

    public void resetHeading(){
        imu.reset();
    }

    public double getEncoderDistance(TalonFXSensorCollection encoder) {
        return encoder.getIntegratedSensorPosition() / 2048 * Constants.Drivetrain.inchesPerRotation;
    }

    public double getLeftEncoderDistance() {
        return m_leftEncoder.getIntegratedSensorPosition() / 2048 * Constants.Drivetrain.inchesPerRotation;
    }
    public double getRightEncoderDistance() {
        return m_rightEncoder.getIntegratedSensorPosition() / 2048 * Constants.Drivetrain.inchesPerRotation;
    }
    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (getEncoderDistance(m_leftEncoder) - getEncoderDistance(m_rightEncoder)) / 2.0;
    }

    public double getEncoderRate(TalonFXSensorCollection encoder) {
        return encoder.getIntegratedSensorVelocity() / 204.8 * Constants.Drivetrain.inchesPerRotation;
    }

    /**
     * Build the Shuffleboard Choosers
     * 
     * @param builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Heading", this::getGyroHeading, null);
        builder.addDoubleProperty("Encoder Distance", this::getAverageEncoderDistance, null);
        builder.addDoubleProperty("Left Encoder Distance", this::getLeftEncoderDistance, null);
        builder.addDoubleProperty("Right Encoder Distance", this::getRightEncoderDistance, null);
        super.initSendable(builder);
    }

    /**
     * Drive based on the PID settings
     */
    @Override
    public void periodic() {
        m_odometry.update(new Rotation2d(Math.toRadians(-getGyroHeading()/2)), getEncoderDistance(m_leftEncoder),
                -getEncoderDistance(m_rightEncoder));
        /*
         * if (headingPidEnabled) { double output =
         * -headingPidController.calculate(getHeading()); double spdLmt = 0.25; if
         * (output < -spdLmt) { output = -spdLmt; } else if (output > spdLmt) { output =
         * spdLmt; } SmartDashboard.putNumber("PID Output", output); drive(output,
         * -output); }
         */
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getEncoderRate(m_leftEncoder), getEncoderRate(m_rightEncoder));
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        zeroHeading();
        m_odometry.resetPosition(pose, new Rotation2d(Math.toRadians(-getGyroHeading())));
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        m_leftEncoder.setIntegratedSensorPosition(0, 30);
        m_rightEncoder.setIntegratedSensorPosition(0, 30);
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more
     * slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        m_drive.setMaxOutput(maxOutput);
    }
}