package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private int selection;

    public AutoGroup(int selection) {
        this.selection = selection;

        switch (this.selection) {
            /* Trench Run */
            case 1: addCommands(
                        new DriveToPoint(0, -216),
                        new DriveToPoint(0, -216)
                    );
                    break;
            /* Center to Mid */
            case 2: addCommands(
                        new DriveToPoint(0, -105),
                        new DriveToPoint(-72, 0),
                        new DriveToPoint(-105, 0)
                    );

            /* Left to Mid */
            // case 3: addCommands(
                // new DriveToPoint(0, -115),
                // new DriveToPoint(0, -115)
            // );
        }
    
    }
}