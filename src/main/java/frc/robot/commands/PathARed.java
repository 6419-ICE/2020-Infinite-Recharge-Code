/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.RobotContainer.drivetrain;

public class PathARed extends CommandBase {
  /**
   * Creates a new PathARed.
   */
  public PathARed() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addRequirements(drivetrain);
    drivetrain.setMaxOutput(.5);
  }

  @Override
  public void initialize(){
    drivetrain.resetEncoders();
    drivetrain.resetHeading();
    drivetrain.diffStop();
  }

  @Override
  public void execute(){
    drivetrain.arcadeDrive(.5, 0);
  }

  @Override
  public void end(final boolean interrupted){
    drivetrain.diffStop();
    
  }

  @Override 
  public boolean isFinished(){
    return drivetrain.getAverageEncoderDistance() >= 20;
  }
}
