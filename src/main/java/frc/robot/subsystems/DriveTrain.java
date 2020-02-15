package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    public ADIS16448_IMU imu;

    public static double kI, kP, kD, kF;

    private TalonFX left1, left2, left3, right1, right2, right3;

    private PIDController headingPidController;
    private boolean headingPidEnabled;
    
    public DriveTrain(){
        imu = new ADIS16448_IMU();
        imu.calibrate();

        left1 = new TalonFX(Constants.FRONT_ONE_PIN);
        left2 = new TalonFX(Constants.FRONT_TWO_PIN);
        left3 = new TalonFX(Constants.FRONT_THREE_PIN);
        right1 = new TalonFX(Constants.BACK_ONE_PIN);
        right2 = new TalonFX(Constants.BACK_TWO_PIN);
        right3 = new TalonFX(Constants.BACK_THREE_PIN);

        left2.follow(left1);
        left3.follow(left1);

        right2.follow(right1);
        right3.follow(right1);

        kP = 0.05;
        kI = 0;
        kD = 0.35;
        kF = 0;

        /* Config PID values: Config 0 */
        left1.config_kP(0, kP); // 0 is the slot index for this current PID config
        left1.config_kI(0, kI);
        left1.config_kD(0, kD);
        left1.config_kF(0, kF);

        right1.config_kP(0, kP); // 0 is the slot index for this current PID config
        right1.config_kI(0, kI);
        right1.config_kD(0, kD);
        right1.config_kF(0, kF);

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

        Preferences prefs = Preferences.getInstance();

        if (!prefs.containsKey("Heading P")) {
            prefs.putDouble("Heading P", 0.045);
        }
        if (!prefs.containsKey("Heading D")) {
            prefs.putDouble("Heading D", 0);
        }

        headingPidController = new PIDController(
            prefs.getDouble("Heading P", 0.045),
            0,
            prefs.getDouble("Heading D", 0));
        headingPidController.setTolerance(8);
    }

    @Override
    public void periodic() {
        if (headingPidEnabled) {
            double output = headingPidController.calculate(getHeading());
            SmartDashboard.putNumber("PID Output", output);
            drive(output, -output);
        }
    }

    public TalonFX getLeftMotors() {
        return left1;
    }

    public TalonFX getRightMotors() {
        return right1;
    }

    public void drive(double l, double r) {
        left1.set(TalonFXControlMode.PercentOutput, l * Constants.speedLmt);
        right1.set(TalonFXControlMode.PercentOutput,-r * Constants.speedLmt);
    }

    public void arcadeDrive(double p, double t) {
        double powValue = p * -1;
        double turnValue = t ; // Do not need to invert turn input
        drive(Constants.speedLmt * (turnValue + powValue), -Constants.speedLmt * (turnValue - powValue));
    }

    public void stop() {
        left1.set(TalonFXControlMode.PercentOutput, 0.0);
        right1.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void syncPIDTunings() {
        Preferences prefs = Preferences.getInstance();
        headingPidController.setPID(
            prefs.getDouble("Heading P", 0.015),
            0,
            prefs.getDouble("Heading D", 0));
    }

    public void setSetpoints(double left, double right){
        left1.set(ControlMode.Position, left/Constants.inchesPerRotation);
        right1.set(ControlMode.Position, -right/Constants.inchesPerRotation);
    }

    public void setHeadingPidEnabled (boolean enabled){
        headingPidEnabled = enabled;
    }

    public void setHeadingTarget(double target){
        headingPidController.setSetpoint(target);
    }

    public double getHeadingTarget(){
        return headingPidController.getSetpoint();
    }

    public boolean atHeadingTarget() {
        return headingPidController.atSetpoint();
    }

    public double getHeading() {
        return imu.getAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Heading", this::getHeading, null);
        builder.addDoubleProperty("Target Heading", this::getHeadingTarget, this::setHeadingTarget);
        builder.addBooleanProperty("At Target Heading", this::atHeadingTarget, null);
        super.initSendable(builder);
    }
}