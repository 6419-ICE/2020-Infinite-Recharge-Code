/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.indexer;
import static frc.robot.RobotContainer.loader;

public class HandleIndexer extends CommandBase {

  private boolean lastLemonState;
  private double fallingEdgeTimestamp;
  private double risingEdgeTimestamp;
  private boolean indexFault;

  /**
   * Creates a new HandleIndexer.
   */
  public HandleIndexer() {
    addRequirements(indexer, loader);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexer.stopIndexer();
    indexFault = false;
    lastLemonState = false;
    risingEdgeTimestamp = fallingEdgeTimestamp = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Add a buffer to the lemon sensor so that the indexer runs longer 
     * even after a lemon is no longer detected
     */
    if (indexFault) {
      indexer.stopIndexer();
      loader.stopLoader();
    } else {
      if (indexer.isLemonPresent()) {
        if (!lastLemonState) {
          risingEdgeTimestamp = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - risingEdgeTimestamp > 3) {
          DriverStation.reportWarning("Indexer fault detected. Lemon jammed in indexer.", false);
          indexFault = true;
        }
        indexer.runIndexer();
        loader.runLoader();
      } else {
        if (lastLemonState) {
          fallingEdgeTimestamp = Timer.getFPGATimestamp();
        }
        if (Timer.getFPGATimestamp() - fallingEdgeTimestamp > 0.1) {
          indexer.stopIndexer();
          loader.stopLoader();
        }
      }
    }

    lastLemonState = indexer.isLemonPresent();
  }

  /* 
  if (indexer.isLemonPresent()) {
      timer.reset();
      indexer.runIndexer();
      isRunning = true;
    } else {
      if (isRunning) {
        timer.start();
        if (timer.get() > 0.1) {
          indexer.stopIndexer();
          timer.stop(); 
          isRunning = false;
        }
      }
    }
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
