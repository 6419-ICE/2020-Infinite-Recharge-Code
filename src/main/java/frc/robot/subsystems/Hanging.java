/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanging extends SubsystemBase {
  /**
   * Creates a new Hanging.
   */

  private DoubleSolenoid leftLiftSolenoid;
  private DoubleSolenoid rightLiftSolenoid;
  private DoubleSolenoid positioningSolenoid;

  public Hanging() {
    leftLiftSolenoid = new DoubleSolenoid(Constants.Hanger.LEFT_SOLENOID_1, Constants.Hanger.LEFT_SOLENOID_2);
    rightLiftSolenoid = new DoubleSolenoid(Constants.Hanger.RIGHT_SOLENOID_1, Constants.Hanger.RIGHT_SOLENOID_2);
    positioningSolenoid = new DoubleSolenoid(Constants.Hanger.POSITIONING_SOLENOID_1, Constants.Hanger.POSITIONING_SOLENOID_2);
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void activateLift(Value m){
    leftLiftSolenoid.set(m);
    rightLiftSolenoid.set(m);
  }

  public void positionLift(Value m){
    positioningSolenoid.set(m);
  }

  public Value liftPosition(){
    return positioningSolenoid.get();
  }

  public Value liftValue(){
    return leftLiftSolenoid.get();
  }

}
