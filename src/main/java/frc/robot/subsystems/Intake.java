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

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

  private VictorSPX intakeMotor;

  public Intake() {
    intakeMotor = new VictorSPX(Constants.INTAKE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public VictorSPX getIntakeMotor(){
    return intakeMotor;
  }

  public void runIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 1);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
