/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveToPoint extends SequentialCommandGroup {
  /**
   * Creates a new DriveToPoint.
   */
  public DriveToPoint(double x, double y) {
    double heading = Math.toDegrees(Math.atan2(y, x));
    double distance = Math.hypot(x, y);

    addCommands(
      new Turn(-heading + 90),
      new DriveByEncoder(distance)
    );
  }
}
