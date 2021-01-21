/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {
  
  private CANSparkMax loaderMotor;
  private DigitalInput loadSensor;

  public Loader() {
    loaderMotor = new CANSparkMax(Constants.LOADER, CANSparkMaxLowLevel.MotorType.kBrushless);
    loaderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    loadSensor = new DigitalInput(Constants.Loader.LOAD_SENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    loaderMotor.set(power);
  }

  public void runLoader(){
    loaderMotor.set(1);
  }

  public void stopLoader(){
    loaderMotor.set(0);
  }

  public boolean isLemonPresent() {
    return !loadSensor.get();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    setName("Loader");
    builder.addBooleanProperty("Lemon Present", this::isLemonPresent, null);
    super.initSendable(builder);
  }
}
