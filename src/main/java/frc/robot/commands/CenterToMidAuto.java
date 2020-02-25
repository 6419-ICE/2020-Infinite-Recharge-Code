package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CenterToMidAuto extends SequentialCommandGroup {

    public CenterToMidAuto() {
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
