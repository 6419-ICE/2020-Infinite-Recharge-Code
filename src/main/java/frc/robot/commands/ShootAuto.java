package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ShootAuto extends SequentialCommandGroup {

    public ShootAuto() {
        addCommands(
                //new DriveToPoint(0, 24),
                new TurretFire().withTimeout(7));
    }
}
