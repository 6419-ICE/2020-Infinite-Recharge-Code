/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeftMidAuto extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public LeftMidAuto() {
        addCommands(
            new TurretFire().withTimeout(5),
            parallel(
                new SetIntakePower(1),
                sequence(
                    new DriveToPoint(0, -150),
                    new DriveToPoint(10, -50),
                    new DriveToPoint(-50, 36),
                    new DriveToPoint(90, 0),
                    new TurretFire().withTimeout(5)
                )
            )
        );
    }
}