package frc.robot.subsystems;

// Indexer - Part of FRC 6419's 2020 codebase
// Generated 20200206:1757 [2/6/2020:17:57]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Indexer extends SubsystemBase {

    private DigitalInput input;

    public Indexer() {

    }

    @Override
    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        setName("Indexer");
        super.initSendable(builder);
    }
}