package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Creates a new HandleDriveTrain Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoGroup() {
        /* Trench Run */
        addCommands(new Turn(160));
        addCommands(new DriveBySeconds(3.0, false));
        addCommands(new DriveBySeconds(3.0, true));
        addCommands(new Turn(160));
    }
}