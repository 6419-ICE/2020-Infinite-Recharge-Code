/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
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
                            joy2 = 1;

    /* Joystick Buttons */
    public static final int shooterButton = 1;

    /* Default Autonomous Values */
    public static final double autoAngle = 90,
                                d1 = 40,
                                d2 = 0;

    /* Hatch Panel Arm Fixed Power */
    public static final double armPower = 1.0;

    /* Hall Effect Sensor Port */
    public static final int sensorPrt = 0;

    /* Universal Motor Speed Limit */
    public static final double speedLmt = 0.74;

    /* Drive Train Encoder Values */
    public static final double rotations = .115;
    public static final double inchesPerRotation = 6 * Math.PI * rotations;
    //public static final double rotationsPerInch = 1.0/InchesPerRotation;

	public static class Turret {
        public static final double rotations = 0.0;
        public static final double inchesPerRotation = 0.0;

        public static final int SHOOTER0 = 7,
                                SHOOTER1 = 8,
                                TRAVERSE = 9;

        public static final double FIRING_SPEED = 5000,
                                   SLEW_SPROCKET_TEETH = 112,
                                   PINION_SPROCKET_TEETH = 12,
                                   ENCODER_TICKS_PER_REV = 4096,
                                   // How far the turret can traverse, in degrees, from *center*
                                   TRAVERSE_LIMIT_ANGLE = 90,
                                   TRAVERSE_SOFT_LIMIT =
                                           (TRAVERSE_LIMIT_ANGLE / 360.0) // limit in revolutions of the turret
                                           * (SLEW_SPROCKET_TEETH / PINION_SPROCKET_TEETH) // limit in revolutions of the pinion
                                           * ENCODER_TICKS_PER_REV; // limit in ticks

        public static final boolean ENABLE_LIMITS = false;
    }
}
