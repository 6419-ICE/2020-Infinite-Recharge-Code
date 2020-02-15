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

    public static class Drivetrain {
        public static final String KP_TAG = "Drivetrain Heading KP",
                                   KI_TAG = "Drivetrain Heading KI",
                                   KD_TAG = "Drivetrain Heading KD";
        public static final int LEFT0 = 4,
                                LEFT1 = 5,
                                LEFT2 = 6,
                                RIGHT0 = 1,
                                RIGHT1 = 2,
                                RIGHT2 = 3;
    }

    public static class Turret {
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

    public static final String VISION_KP_TAG = "Vision KP",
                               VISION_KI_TAG = "Vision KI",
                               VISION_KD_TAG = "Vision KD";
}
