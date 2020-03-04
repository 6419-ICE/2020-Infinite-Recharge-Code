/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /**
   * Creates a new indexing.
   */
  private VictorSPX indexingMotor;
  private ColorSensorV3 loadSensor;

  public Indexer() {
      indexingMotor = new VictorSPX(Constants.INDEXER);
      loadSensor = new ColorSensorV3(I2C.Port.kOnboard);
      indexingMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double power) {
    indexingMotor.set(ControlMode.PercentOutput, power);
  }

  public void runIndexer(){
    indexingMotor.set(ControlMode.PercentOutput, -1);
  }

  public void reverseIndexer(){
    indexingMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopIndexer(){
    indexingMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean isLemonPresent() {
    return loadSensor.getProximity() > 700;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    setName("Indexer");
    builder.addDoubleProperty("Proximity", loadSensor::getProximity, null);
    builder.addBooleanProperty("Is Lemon Present", this::isLemonPresent, null);
    super.initSendable(builder);
  }
}
