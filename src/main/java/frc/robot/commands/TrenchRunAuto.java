/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TrenchRunAuto extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public TrenchRunAuto() {
        addCommands(
            new TurretFire(0).withTimeout(4),
            parallel(
                new SetIntakePower(-1), 
                sequence(
                    new DriveToPoint(0, -170, 0.17),
                    new WaitCommand(.5),
                    new DriveToPoint(0, -170, 0.8)
                )
            )
        );
    }
}