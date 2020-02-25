/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Loader extends SubsystemBase {
  
  private VictorSPX loaderMotor;

  public Loader() {
    loaderMotor = new VictorSPX(Constants.LOADER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public VictorSPX getLoaderMotor(){
    return loaderMotor;
  }

  public void setPower(double power) {
    loaderMotor.set(ControlMode.PercentOutput, power);
  }

  public void runLoader(){
    loaderMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopLoader(){
    loaderMotor.set(ControlMode.PercentOutput, 0);
  }
}
