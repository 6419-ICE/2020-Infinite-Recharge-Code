/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Drive to an x-y position based on a plane mapped by the encoders */
public class DriveToPoint extends SequentialCommandGroup {

  public DriveToPoint(double x, double y) {
    this(x, y, 1);
  }

  /**
   * Creates a new DriveToPoint.
   */
  public DriveToPoint(double x, double y, double speedLimit) {
    /* calculate the position in encoder and angle values */
    double heading = Math.toDegrees(Math.atan2(y, x));
    double distance = Math.hypot(x, y);

    /* Use Turn and DriveByEncoder commands with specified headings and distances */
    addCommands(
      new Turn(-heading + 90),
      new DriveByEncoder(distance, speedLimit)
    );
  }
}
