package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class DriveTrain extends PIDSubsystem{

    public ADIS16448_IMU imu;

    public static double kI, kP, kD, kF;

    private CANSparkMax left1, left2, left3, right1, right2, right3;
    public CANEncoder   motorEncoderL1,
                        motorEncoderL2,
                        motorEncoderL3,
                        motorEncoderR1,
                        motorEncoderR2,
                        motorEncoderR3;    

    private double rotations = .204;
    private double InchesPerRotation = 6 * Math.PI * rotations;

    public CANPIDController leftController,
                            rightController;
    
    public DriveTrain(){
        super(new PIDController(0.04, 0, 0.08, 0));
        getController().setTolerance(0.1);
        enable();
        imu = new ADIS16448_IMU();
        imu.calibrate();

        left1 = new CANSparkMax(Constants.FRONT_ONE_PIN, MotorType.kBrushless);
        left2 = new CANSparkMax(Constants.FRONT_TWO_PIN, MotorType.kBrushless);
        left3 = new CANSparkMax(Constants.FRONT_THREE_PIN, MotorType.kBrushless);
        right1 = new CANSparkMax(Constants.BACK_ONE_PIN, MotorType.kBrushless);
        right2 = new CANSparkMax(Constants.BACK_TWO_PIN, MotorType.kBrushless);
        right3 = new CANSparkMax(Constants.BACK_THREE_PIN, MotorType.kBrushless);

        motorEncoderL1 = left1.getEncoder();
        motorEncoderL2 = left2.getEncoder();
        motorEncoderL3 = left3.getEncoder();
        motorEncoderR1 = right1.getEncoder();
        motorEncoderR2 = right2.getEncoder();
        motorEncoderR3 = right3.getEncoder();

        leftController = left1.getPIDController();
        rightController = right1.getPIDController();
        leftController.setOutputRange(-1, 1);
        rightController.setOutputRange(-1, 1);
        
        
        left2.follow(left1);
        left3.follow(left1);

        right2.follow(right1);
        right3.follow(right1);
    }

    @Override
    public void periodic() {

    }
        
    public void drive(double l, double r) {
        left1.set(l * 0.4);
        right1.set(-r * 0.4);
    }

    public void stop() {
        left1.set(0.0);
        right1.set(0.0);
    }



    @Override
    protected double getMeasurement() {
        return motorEncoderL1.getPosition() * InchesPerRotation;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        SmartDashboard.putNumber("PID Output", output);
        drive(output*.5, -output*.5);
    }
    public void enablePID (boolean enabled){
        if(enabled){
            enable();
        } else if(!enabled){
            disable();
        }
    }

    public void setHeadingTarget(double target){
        getController().setSetpoint(target);
    }

    public double getHeadingTarget(){
        return getController().getSetpoint();
    }

}