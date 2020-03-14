package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TurretClearAndFire extends SequentialCommandGroup {

    public TurretClearAndFire() {
        addCommands(
            parallel(
                new EjectBallFromShooter(),
                new SetIndexerPower(1),
                new SetLoaderPower(-1)
            ).withTimeout(0.2),
            new TurretFire(0)
        );
    }
}