/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;

/** @deprecated DriveBySeconds has been replaced by DriveByEncoder
 * - DriveBySeconds is merely a concept for mapping Autonomous strategies
 * - DriveByEncoder is officially supported for accurate Autonomous travel and delivery
 */
@Deprecated
public class DriveBySeconds extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Timer timer;
    private double timeLimit;
    private boolean done;
    private boolean backwards;

    /** Construct a timer with a limit for running the command and determine 
     * if the robot needs to drive backwards 
     * @param seconds - The time limit to check against the current time
     * @param backwards - Fowards if false, Backwards if true
     */
    public DriveBySeconds(double seconds, boolean backwards) {
        timeLimit = seconds;
        this.backwards = backwards;
        timer = new Timer();
        addRequirements(RobotContainer.drivetrain);
    }

    /** Reset and start the timer. Ensure the drivetrain is stopped */
    @Override
    public void initialize() {
        done = false;
        timer.reset();
        RobotContainer.drivetrain.stop(); // Don't move on init
        timer.start();
    }

    /** Stop the command if the time limit has been passed, otherwise continue driving */
    @Override
    public void execute() {
        if (timer.get() > timeLimit) {
            RobotContainer.drivetrain.stop();
            
            done = true;
        } else {
            if (backwards) {
                RobotContainer.drivetrain.drive(-1, -1);
            } else {
                RobotContainer.drivetrain.drive(1, 1);
            }
        }
        super.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    /** @return done */
    @Override
    public boolean isFinished() {
        return done;
    }
}
