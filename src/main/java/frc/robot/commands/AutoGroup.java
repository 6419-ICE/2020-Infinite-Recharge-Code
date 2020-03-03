/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private String selection;

    /**
     * Construct each Autonomous CommandGroup
     * 
     * @param name - The selected CommandGroup based off of the SendableChooser
     *             current selection
     */
    public AutoGroup(String name) {
        this.selection = name;

        switch (this.selection) {
        /* Trench Run */
        case "AUTO_1":
            addCommands(
                new TurretFire().withTimeout(3),
                new ParallelCommandGroup(
                    new SetIntakePower(1),
                    new SequentialCommandGroup(
                        new DriveToPoint(0, -170),
                        new ParallelCommandGroup(
                            new DriveToPoint(0, -190),
                            new TurretFire().withTimeout(4)
                        )
                    )
                )
            );
            break;
        /* Center to Mid */
        case "AUTO_2":
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
            break;
        /* Left to Mid */
        case "AUTO_3":
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
            break;
        default:
            // Run the Trench Run by default
            addCommands( 
                new DriveToPoint(0, -216), 
                new DriveToPoint(0, -216));
            break;
        }

    }
}