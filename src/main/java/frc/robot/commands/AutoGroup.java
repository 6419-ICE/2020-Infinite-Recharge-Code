package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    public AutoGroup() {
        /* Trench Run */
        
        addCommands(new DriveToPoint(0, -216),
                    new DriveToPoint(0, -216)
        );
    }
}