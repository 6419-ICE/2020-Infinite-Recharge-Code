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

public class Indexer extends SubsystemBase {
  /**
   * Creates a new indexing.
   */
  private VictorSPX indexingMotor;

  public Indexer() {
      indexingMotor = new VictorSPX(Constants.INDEXER);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public VictorSPX getIndexingMotor(){
    return indexingMotor;
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
}
