/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
                public static final double headingPidTolerance = 1.5;

                /*
                 * Trajectory Constants From characterization analyzer Max Acceptable Control
                 * Effort (V)
                 */
                public static final double ksVolts = 0.598, ksVoltsSecondsPerMeter = 0.929,
                                ksVoltsSecondsSquaredPerMeter = 0.0564, kPDriveVel = 0.00000465, kTrackWidth = 0.8304242384791596;

                public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                                kTrackWidth);

                // Max velocity and acceleration of Ramsete
                public static final double kMaxSpeedMetersPerSecond = 4, kMaxAccelerationMetersPerSecondSquared = 1;

                // Larger values of b make convergence more aggressive like a proportional term
                // whereas larger values of zeta provide more damping in the response.
                public static final double kRamseteB = .8, kRamseteZeta = 1.4;

                /*
                 * characterization: Position loop: kP: 0.0152 kD: 0.0859 Velocity loop: kP:
                 * 0.000556 kD: 0
                 */

        }

        /* Motor Ports */
        public static final int FRONT_ONE_PIN = 1, FRONT_TWO_PIN = 2, FRONT_THREE_PIN = 3, BACK_ONE_PIN = 4,
                        BACK_TWO_PIN = 5, BACK_THREE_PIN = 6, SHOOTER_ONE_PIN = 0, SHOOTER_TWO_PIN = 1,
                        TRAVERSE_PIN = 2;

        /* Joystick Ports */
        public static final int joy1 = 0, joy2 = 1, joy3 = 2;

        /* Joystick Buttons */
        public static final int shooterBtn = 9, intakeBtn = 1, intakeIndexBtn = -1, outtakeBtn = 2, indexForward = 6,
                        indexReverse = 7, liftingButton = 5;

        /* Default Autonomous Values */
        public static final double autoAngle = 90, d1 = 40, d2 = 0;

        /* Hatch Panel Arm Fixed Power */
        public static final double armPower = 1.0;

        /* Hall Effect Sensor Port */
        public static final int sensorPrt = 0;

        /* Loader Motor Port */
        public static final int LOADER = 10;

        /* Intake Motor Port */
        public static final int INTAKE = 11;

        /* Intake Motor Port */
        public static final int INDEXER = 12;

        /** Constants for the Turret */
        public static class Turret {
                public static final int SHOOTER0 = 7, SHOOTER1 = 8, TRAVERSE = 9, HOMING_SWITCH = 0;

                public static final double FIRING_SPEED = 5000, SLEW_SPROCKET_TEETH = 112, PINION_SPROCKET_TEETH = 12,
                                ENCODER_TICKS_PER_REV = 4096,
                                // How far the turret can traverse, in degrees, from *center*
                                TRAVERSE_LIMIT_ANGLE = 90,
                                // limit in revolutions of the turret
                                TRAVERSE_SOFT_LIMIT = (TRAVERSE_LIMIT_ANGLE / 360.0)
                                                // limit in revolutions of the pinion
                                                * (SLEW_SPROCKET_TEETH / PINION_SPROCKET_TEETH) * ENCODER_TICKS_PER_REV; // limit
                                                                                                                         // in
                                                                                                                         // ticks

                // note: this enables *hardware* limits. if this is false, the turret uses encoder limits.
                // keep this set to false.
                public static final boolean ENABLE_LIMITS = false;
        }

        public static class Loader {
                public static final int LOADER_MOTOR = 10, LOAD_SENSOR = 1;
        }

        public static class Hanger {
                public static final int COMPRESSOR = 0, LEFT_SOLENOID_1 = 0, LEFT_SOLENOID_2 = 1, RIGHT_SOLENOID_1 = 2,
                                RIGHT_SOLENOID_2 = 3, POSITIONING_SOLENOID_1 = 4, POSITIONING_SOLENOID_2 = 5;

        }
}
