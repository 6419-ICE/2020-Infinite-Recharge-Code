package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoGroup extends SequentialCommandGroup {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private String selection;

    public AutoGroup(String name) {
        this.selection = name;

        switch (this.selection) {
            /* Trench Run */
            case "AUTO_1": 
                addCommands(
                    new DriveToPoint(0, -216),
                    new DriveToPoint(0, -216)
                );
                break;
            /* Center to Mid */
            case "AUTO_2": 
                addCommands(
                    new DriveToPoint(0, -105),
                    new DriveToPoint(-72, 0),
                    new DriveToPoint(-105, 0)
                );
                break;
            /* Left to Mid */
            // case "AUTO_3": addCommands(
                // new DriveToPoint(0, -115),
                // new DriveToPoint(0, -115)
            // );
        }
    
    }
}