package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Creates a new HandleDriveTrain Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoGroup(DriveTrain drive) {
        addCommands(new DriveByEncoder(drive, 200));
    
    }

}

   