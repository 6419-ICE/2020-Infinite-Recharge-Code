package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TurretClearAndFire extends SequentialCommandGroup {

    public TurretClearAndFire() {
        System.out.println("TurretClearAndFire is supposed to work!");
        addCommands(
            parallel(
                new EjectBallFromShooter(),
                new SetIndexerPower(1)
               // new SetLoaderPower(-1)
            ).withTimeout(0.2),
            new TurretFire(0)
        );
    }
}