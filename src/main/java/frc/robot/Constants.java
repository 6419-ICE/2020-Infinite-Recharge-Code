/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Constants for ICE 6419
 */
public final class Constants {

    /** Constants for the Drivetrain */
    public static class Drivetrain {
        /* Universal Motor Speed Limit */
        public static final double speedLmt = 1;

        /* Drive Train Encoder Values */
        public static final double rotations = .115;
        public static final double inchesPerRotation = 6 * Math.PI * rotations;
        // public static final double rotationsPerInch = 1.0/InchesPerRotation;
        public static final double headingPidTolerance = 3;

    }

    /* Motor Ports */
    public static final int FRONT_ONE_PIN = 1, 
                            FRONT_TWO_PIN = 2, 
                            FRONT_THREE_PIN = 3, 
                            BACK_ONE_PIN = 4,
                            BACK_TWO_PIN = 5, 
                            BACK_THREE_PIN = 6, 
                            SHOOTER_ONE_PIN = 0, 
                            SHOOTER_TWO_PIN = 1, 
                            TRAVERSE_PIN = 2;

    /* Joystick Ports */
    public static final int joy1 = 0, 
                            joy2 = 1,
                            joy3 = 2;

<<<<<<< HEAD
=======
    /* Joystick Buttons */
    public static final int shooterBtn = 1,
                            intakeBtn = 5,
                            intakeIndexBtn = 2,
                            outtakeBtn = 3,
                            indexBtn = 1,
                            indexForward = 11,
                            indexReverse = 12;

>>>>>>> 00bf959cec7704e54f64a48e99a0a7f7afbfc20e
    /* Default Autonomous Values */
    public static final double autoAngle = 90, d1 = 40, d2 = 0;

    /* Hatch Panel Arm Fixed Power */
    public static final double armPower = 1.0;

    /* Hall Effect Sensor Port */
    public static final int sensorPrt = 0;

    
    
    /* Loader Motor Port*/
    public static final int LOADER = 10;

    /* Intake Motor Port */
    public static final int INTAKE = 11;
    
    /* Intake Motor Port */
    public static final int INDEXER = 12;

    /** Constants for the Turret */
    public static class Turret {
        public static final int SHOOTER0 = 7, 
                                SHOOTER1 = 8, 
                                TRAVERSE = 9,
                                HOMING_SWITCH = 0;

        public static final double FIRING_SPEED = 5000, 
                                    SLEW_SPROCKET_TEETH = 112, 
                                    PINION_SPROCKET_TEETH = 12,
                                    ENCODER_TICKS_PER_REV = 4096,
                                    // How far the turret can traverse, in degrees, from *center*
                                    TRAVERSE_LIMIT_ANGLE = 90, 
                                    // limit in revolutions of the turret
                                    TRAVERSE_SOFT_LIMIT = (TRAVERSE_LIMIT_ANGLE / 360.0)
                                    // limit in revolutions of the pinion
                                    * (SLEW_SPROCKET_TEETH / PINION_SPROCKET_TEETH)
                                    * ENCODER_TICKS_PER_REV; // limit in ticks

        public static final boolean ENABLE_LIMITS = false;
    }

    public static class Loader {
        public static final int LOADER_MOTOR = 10,
                                LOAD_SENSOR = 3;
    }
}
