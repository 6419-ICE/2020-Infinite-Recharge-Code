package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Creates a new HandleDriveTrain Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoGroup() {
        addCommands(new DriveBySeconds(RobotContainer.drivetrain, 5)
                    /*, new DriveBySeconds(5) */);
    
    }

}

   