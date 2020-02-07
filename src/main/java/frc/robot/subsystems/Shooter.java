/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterMotorRight;
  private CANSparkMax shooterMotorLeft; // Left requires negative power
  
  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    shooterMotorLeft = new CANSparkMax(Constants.SHOOTER_TWO_PIN, MotorType.kBrushless);
    shooterMotorRight = new CANSparkMax(Constants.SHOOTER_ONE_PIN, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot(double p){
    shooterMotorLeft.set(-p);
    shooterMotorRight.set(p);
  }
}
