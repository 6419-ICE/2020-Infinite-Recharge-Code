/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterMidAuto extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public CenterMidAuto() {
        addCommands(
            new TurretFire().withTimeout(5),
            parallel(
            new SetIntakePower(1),
                sequence(
                    new DriveToPoint(0, -105),
                    new DriveToPoint(-72, 0),
                    new DriveToPoint(-105, 0),
                    new TurretFire().withTimeout(5)
                )
            )
        );
    }
}