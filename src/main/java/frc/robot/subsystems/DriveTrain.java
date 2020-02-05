package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

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
    public CANEncoder   motorEncoderL1,
                        motorEncoderL2,
                        motorEncoderL3,
                        motorEncoderR1,
                        motorEncoderR2,
                        motorEncoderR3;    

    


    public CANPIDController leftController,
                            rightController;

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

        /* CHANGES WITH TALONFX:

        Gets the position of the encoder:
        left1.getSelectedSensorPosition();

        Set the target position for the PID controller
        left1.set(ControlMode.Position, target);

        */

        /*motorEncoderL1 = left1
        motorEncoderL2 = left2.getEncoder();
        motorEncoderL3 = left3.getEncoder();
        motorEncoderR1 = right1.getEncoder();
        motorEncoderR2 = right2.getEncoder();
        motorEncoderR3 = right3.getEncoder();

        leftController = left1.getPIDController();
        rightController = right1.getPIDController();

        leftController.setOutputRange(-0.5, 0.5);
        rightController.setOutputRange(-0.5, 0.5);

        kP = 0.05;
        kI = 0;
        kD = 0.35;
        kF = 0;

        leftController.setP(kP);
        leftController.setI(kI);
        leftController.setD(kD);
        leftController.setFF(kF);

        rightController.setP(kP);
        rightController.setI(kI);
        rightController.setD(kD);
        rightController.setFF(kF);
        
        leftController.setSmartMotionMaxAccel(3000, 0);
        leftController.setSmartMotionMaxVelocity(5000, 0);
        leftController.setSmartMotionMinOutputVelocity(60, 0);

        rightController.setSmartMotionMaxAccel(3000, 0);
        rightController.setSmartMotionMaxVelocity(5000, 0);
        rightController.setSmartMotionMinOutputVelocity(60, 0);
        */
        
        left2.follow(left1);
        left3.follow(left1);

        right2.follow(right1);
        right3.follow(right1);

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

    public double getInchesPerRotation() {
        return Constants.inchesPerRotation;
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
        leftController.setReference(left/Constants.inchesPerRotation, ControlType.kPosition);
        rightController.setReference(-right/Constants.inchesPerRotation, ControlType.kPosition);
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