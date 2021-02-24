package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    public ADIS16448_IMU imu;

    public static double kI, kP, kD, kF;

    private TalonFX left1, left2, left3, right1, right2, right3;
    private WPI_TalonFX WPI_left1, WPI_left2, WPI_left3, WPI_right1, WPI_right2, WPI_right3;
    private TalonFXSensorCollection left_encoder, right_encoder;

    private SpeedControllerGroup leftGroup, rightGroup;

    private DifferentialDrive diff_drive;
    private DifferentialDriveOdometry drive_odometry;

    private PIDController headingPidController;
    private boolean headingPidEnabled;
    private Gyro mGyro;
    public DriveTrain(){
        /* Reset the Accelerometer/Gyro/etc. */
        imu = new ADIS16448_IMU();
        imu.setYawAxis(ADIS16448_IMU.IMUAxis.kZ);
        imu.calibrate();
        mGyro = imu;


        WPI_left1 = new WPI_TalonFX(Constants.FRONT_ONE_PIN);
        WPI_left2 = new WPI_TalonFX(Constants.FRONT_TWO_PIN);
        WPI_left3 = new WPI_TalonFX(Constants.FRONT_THREE_PIN);
        WPI_right1 = new WPI_TalonFX(Constants.BACK_ONE_PIN);
        WPI_right2 = new WPI_TalonFX(Constants.BACK_TWO_PIN);
        WPI_right3 = new WPI_TalonFX(Constants.BACK_THREE_PIN);

        leftGroup = new SpeedControllerGroup(WPI_left1, WPI_left2, WPI_left3);
        rightGroup = new SpeedControllerGroup(WPI_right1, WPI_right2, WPI_right3);

        leftGroup.setInverted(false);
        rightGroup.setInverted(false);

        WPI_left1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        WPI_right1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

        left_encoder = WPI_left1.getSensorCollection();
        right_encoder = WPI_right1.getSensorCollection();

        diff_drive = new DifferentialDrive(leftGroup, rightGroup);

        resetEncoders();
        drive_odometry = new DifferentialDriveOdometry(mGyro.getRotation2d());



        /* Six-motor Falcon 500 Drivetrain */
        left1 = new TalonFX(Constants.FRONT_ONE_PIN);
        left2 = new TalonFX(Constants.FRONT_TWO_PIN);
        left3 = new TalonFX(Constants.FRONT_THREE_PIN);
        right1 = new TalonFX(Constants.BACK_ONE_PIN);
        right2 = new TalonFX(Constants.BACK_TWO_PIN);
        right3 = new TalonFX(Constants.BACK_THREE_PIN);

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

        left1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);// Try Absolute instead of relative
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

        /* CHANGES WITH TALONFX:

        Gets the position of the encoder:
        left1.getSelectedSensorPosition();

        /* For conversion when using Motion Profiling:
        leftController.setSmartMotionMaxAccel(3000, 0);
        leftController.setSmartMotionMaxVelocity(5000, 0);
        leftController.setSmartMotionMinOutputVelocity(60, 0);

        rightController.setSmartMotionMaxAccel(3000, 0);
        rightController.setSmartMotionMaxVelocity(5000, 0);
        rightController.setSmartMotionMinOutputVelocity(60, 0);
        */

        /* Change PID in Shuffleboard */
        Preferences prefs = Preferences.getInstance();

        if (!prefs.containsKey("Heading P")) {
            prefs.putDouble("Heading P", 0.045);
        }
        if (!prefs.containsKey("Heading D")) {
            prefs.putDouble("Heading D", 0);
        }

        headingPidController = new PIDController(
            0.0061,//.00435 for 50%, .0061 for 25%
            0,
            0.00001);
        headingPidController.setTolerance(Constants.Drivetrain.headingPidTolerance);
    }


    /* Drive based on the PID settings */
    @Override
    public void periodic() {
        drive_odometry.update(mGyro.getRotation2d(), getEncoderDistance(left_encoder), -getEncoderDistance(right_encoder));
/*
        if (headingPidEnabled) {
            double output = -headingPidController.calculate(getHeading());
            double spdLmt = 0.25;
            if (output < -spdLmt) {
                output = -spdLmt;
            } else if (output > spdLmt) {
                output = spdLmt;
            }
            SmartDashboard.putNumber("PID Output", output);
            drive(output, -output);
        }
        */
    }

    public Pose2d getPose(){
        return drive_odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(getEncoderRate(left_encoder), getEncoderRate(right_encoder));
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        drive_odometry.resetPosition(pose, mGyro.getRotation2d());
    }

    public void setMaxOutput(double maxOutput){
        diff_drive.setMaxOutput(maxOutput);
    }

    public void setMaxMotorSpeed(double speed) {
        left1.configPeakOutputForward(speed);
        left1.configPeakOutputReverse(-speed);
        right1.configPeakOutputForward(speed);
        right1.configPeakOutputReverse(-speed);
    }

    /** @return left motors for use in Autonomous commands */
    public TalonFX getLeftMotors() {
        return left1;
    }

    /** @return right motors for use in Autonomous commands */
    public TalonFX getRightMotors() {
        return right1;
    }

    /** Set each set of motors to a given power percentage 
     * @param l - Left Power
     * @param r - Right Power
    */
    public void drive(double l, double r) {
        left1.set(TalonFXControlMode.PercentOutput, l * Constants.Drivetrain.speedLmt);
        right1.set(TalonFXControlMode.PercentOutput, -r * Constants.Drivetrain.speedLmt);
    }

    /** WPI MOTOR ARCADE DRIVE
     *  Set each set of motors to a given power percentage based on Joystick axes 
     * This is strictly used to set motors during Teleop
     * @param p - Left joystick input (throttle power)
     * @param t - Right joystick input (turn power)
    */
    public void arcadeDrive(double fwd, double rot){
        diff_drive.arcadeDrive(fwd, rot);
    }

    /** WPI MOTOR TANK DRIVE VOLTAGE
     *  Set each set of motors to a given voltage based on Joystick axes 
     * This is strictly used to set motors during Teleop
     * @param p - Left joystick input (throttle power)
     * @param t - Right joystick input (turn power)
    */
    public void tankDriveVolts(double leftVoltage, double rightVoltage){
        leftGroup.setVoltage(leftVoltage);
        rightGroup.setVoltage(rightVoltage);
        diff_drive.feed();
    }

    /** CTRE MOTOR ARCADE DRIVE
     *  Set each set of motors to a given power percentage based on Joystick axes 
     * This is strictly used to set motors during Teleop
     * @param p - Left joystick input (throttle power)
     * @param t - Right joystick input (turn power)
    */
    /*public void arcadeDrive(double p, double t) {
        double powValue = p * -1;
        double turnValue = t ; // Do not need to invert turn input
        drive(Constants.Drivetrain.speedLmt * (turnValue + powValue), -Constants.Drivetrain.speedLmt * (turnValue - powValue));
    }*/

    /** Stop the motors entirely */
    public void stop() {
        left1.set(TalonFXControlMode.PercentOutput, 0.0);
        right1.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void diffStop() {
        diff_drive.arcadeDrive(0, 0);
    }

    /** Refresh PID with Shuffleboard tunings */
    public void syncPIDTunings() {
        Preferences prefs = Preferences.getInstance();
        headingPidController.setPID(
            prefs.getDouble("Heading P", 0.015),
            0,
            prefs.getDouble("Heading D", 0));
    }

    /** Drive to a set of points by Encoder Drive
     * @param left - Left point
     * @param right - Right point
     */
    public void setSetpoints(double left, double right){
        left1.set(TalonFXControlMode.Position, left/Constants.Drivetrain.inchesPerRotation/2 * 4096);
        right1.set(TalonFXControlMode.Position, -right/Constants.Drivetrain.inchesPerRotation/2 * 4096);
        
    }

    /** Specify wether to enable or disable headingPID
     * @param enabled - Enable or disable
     */
    public void setHeadingPidEnabled (boolean enabled){
        headingPidEnabled = enabled;
    }

    /** Set the target heading to turn to
     * @param target - The specified target heading
     */
    public void setHeadingTarget(double target){
        headingPidController.setSetpoint(target);
    }

    /** @return the target heading */
    public double getHeadingTarget(){
        return headingPidController.getSetpoint();
    }

    /** @return if the target heading has been reached */
    public boolean atHeadingTarget() {
        return headingPidController.atSetpoint();
    }

    /** @return the current heading from the IMU */
    public double getHeading() {
        return imu.getAngle();
    }

    public double getGyroHeading(){
        return mGyro.getRotation2d().getDegrees();
    }

    public double getTurnRate(){
        return -mGyro.getRate();
    }

    public TalonFXSensorCollection getLeftEncoder(){
        return left_encoder;
    }

    public TalonFXSensorCollection getRightEncoder(){
        return right_encoder;
    }

    public void resetHeading(){
        imu.reset();
    }

    public void zeroHeading(){
        imu.reset();
        mGyro.reset();
    }

    public void resetEncoders(){
        left_encoder.setIntegratedSensorPosition(0, 30);
        right_encoder.setIntegratedSensorPosition(0, 30);
    }

    public double getEncoderDistance(TalonFXSensorCollection encoder){
        return encoder.getIntegratedSensorPosition()/2048 * Constants.Drivetrain.inchesPerRotation;
    }

    public double getAverageEncoderDistance(){
        return (getEncoderDistance(left_encoder) - getEncoderDistance(right_encoder)) / 2.0;
    }

    public double getEncoderRate(TalonFXSensorCollection encoder){
        return encoder.getIntegratedSensorVelocity()/204.8 * Constants.Drivetrain.inchesPerRotation;
    }

    /** Build the Shuffleboard Choosers
     * @param builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Heading", this::getGyroHeading, null);
        builder.addDoubleProperty("Encoder Distance", this::getAverageEncoderDistance, null);
        super.initSendable(builder);
    }
}   