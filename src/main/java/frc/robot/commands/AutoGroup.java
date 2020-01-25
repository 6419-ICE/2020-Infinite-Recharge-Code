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
        //addCommands(new Turn(180));
        //addCommands(new DriveByEncoder(200));
        //addCommands(new DriveByEncoder(-20));
        //addCommands(new Turn(160));
        
        addCommands(new DriveToPoint(0, -200));
    }
}