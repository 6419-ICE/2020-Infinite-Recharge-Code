package frc.robot.subsystems;

// Limelight - Part of FRC 6419's 2020 codebase
// Generated 20200110:1237 [1/10/2020:12:37]
// by fortr.
// (c) 2020 FRC 6419 "ICE", all rights reserved.

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class Limelight extends SubsystemBase {

    public enum LightMode {
        DEFAULT,
        OFF,
        BLINK,
        ON;


        @Override
        public String toString() {
            switch (this) {
                default:
                case DEFAULT:
                    return "Default";
                case OFF:
                    return "Off";
                case BLINK:
                    return "Blink";
                case ON:
                    return "On";
            }
        }

        public static LightMode forIndex(int index) {
            switch (index) {
                case 0:
                    return DEFAULT;
                case 1:
                    return OFF;
                case 2:
                    return BLINK;
                case 3:
                    return ON;
            }
            throw new IllegalArgumentException("Index out of range");
        }
    }

    public enum CameraMode {
        VISION,
        DRIVER_CAMERA;


        @Override
        public String toString() {
            switch (this) {
                default:
                case VISION:
                    return "Vision";
                case DRIVER_CAMERA:
                    return "Driver Camera";
            }
        }

        public static CameraMode forIndex(int index) {
            switch (index) {
                case 0:
                    return VISION;
                case 1:
                    return DRIVER_CAMERA;
            }
            throw new IllegalArgumentException("Index out of range");
        }
    }

    private NetworkTableEntry horizontalAngle, verticalAngle;
    private NetworkTableEntry ledMode,
            camMode,
            tv,
            ta;

    private SendableChooser<LightMode> lightModeSendableChooser;
    private SendableChooser<CameraMode> cameraModeSendableChooser;

    public Limelight() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        horizontalAngle = table.getEntry("tx");
        verticalAngle = table.getEntry("ty");
        ledMode = table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        tv = table.getEntry("tv");
        ta = table.getEntry("ta");
        setCameraMode(CameraMode.VISION);
        setLightMode(LightMode.ON);
    }

    public double getHorizontalAngle() {
        return horizontalAngle.getDouble(0);
    }

    public double getVerticalAngle() {
        return verticalAngle.getDouble(0);
    }

    public boolean canSeeTarget() {
        return tv.getNumber(0).intValue() > 0;
    }

    public double getTargetArea() {
        return ta.getNumber(0).doubleValue() / 100;
    }

    public void setLightMode(LightMode mode) {
        ledMode.setNumber(mode.ordinal());
    }

    public LightMode getLightMode() {
        return LightMode.forIndex(ledMode.getNumber(0).intValue());
    }

    public void setCameraMode(CameraMode mode) {
        ledMode.setNumber(mode.ordinal());
    }

    public CameraMode getCameraMode() {
        return CameraMode.forIndex(camMode.getNumber(0).intValue());
    }

    @Override
    public void periodic() {
        if (lightModeSendableChooser != null) {
            int mode = lightModeSendableChooser.getSelected().ordinal();
            if (!ledMode.getNumber(0).equals(mode)) {
                ledMode.setNumber(mode);
            }
        }
        if (cameraModeSendableChooser != null) {
            int mode = cameraModeSendableChooser.getSelected().ordinal();
            if (!camMode.getNumber(0).equals(mode)) {
                camMode.setNumber(mode);
            }
        }
    }



    @Override
    public void initSendable(SendableBuilder builder) {
        setName("Limelight");
        builder.addDoubleProperty("Horizontal Angle", this::getHorizontalAngle, null);
        builder.addDoubleProperty("Vertical Angle", this::getVerticalAngle, null);
        builder.addBooleanProperty("Can See Target", this::canSeeTarget, null);
        builder.addDoubleProperty("Target Area", this::getTargetArea, null);
        lightModeSendableChooser = new SendableChooser<>();
        lightModeSendableChooser.addOption(LightMode.DEFAULT.toString(), LightMode.DEFAULT);
        lightModeSendableChooser.setDefaultOption(LightMode.OFF.toString(), LightMode.OFF);
        lightModeSendableChooser.addOption(LightMode.BLINK.toString(), LightMode.BLINK);
        lightModeSendableChooser.addOption(LightMode.ON.toString(), LightMode.ON);
        cameraModeSendableChooser = new SendableChooser<>();
        cameraModeSendableChooser.setDefaultOption(CameraMode.DRIVER_CAMERA.toString(), CameraMode.DRIVER_CAMERA);
        cameraModeSendableChooser.addOption(CameraMode.VISION.toString(), CameraMode.VISION);
        super.initSendable(builder);
    }
}