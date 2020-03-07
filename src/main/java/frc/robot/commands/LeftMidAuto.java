package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LeftMidAuto extends SequentialCommandGroup {

    public LeftMidAuto() {
        addCommands(
                new TurretFire(0).withTimeout(5),
                parallel(
                        new SetIntakePower(1),
                        sequence(
                                new DriveToPoint(0, -150),
                                new DriveToPoint(10, -50),
                                new DriveToPoint(-50, 36),
                                new DriveToPoint(90, 0),
                                new TurretFire(0).withTimeout(5)
                        )
                )
        );
    }
}