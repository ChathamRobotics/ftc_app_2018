package org.firstinspires.ftc.team9853;

/*!
 * FTC_APP_2018
 * Copyright (c) 2017 Chatham Robotics
 * MIT License
 *
 * @Last Modified by: storm
 * @Last Modified time: 9/17/2017
 */


import android.support.annotation.NonNull;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.common.robot.Robot;
import org.chathamrobotics.common.systems.Gripper;
import org.chathamrobotics.common.systems.GyroHandler;
import org.chathamrobotics.common.systems.HolonomicDriver;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/**
 * Team 9853's robot
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Robot9853 extends Robot {

    private static final int GYRO_TOERANCE = 15;
    private static final double GYRO_TOERANCE_RAD = Math.toRadians(25);
    private static final int GYRO_SCALE_FACTOR = 2;
    private static final double[] LEFT_ARM_POSITIONS = {0.0, 1.0}; //up, down
    private static final double[] RIGHT_ARM_POSITIONS = {1.0, 0.0}; // up, down
    private static final String VUFORIA_LICENSE_KEY = "AeTwV0H/////AAAAGfe7ayWmjE9+nI9k65aoO+NQIIujZBIX8AxeoVDf9bwLLNvQ6QwvM+Clc3CE/8Pumv5guDuXMxkERpyJTzSb50PcrH9y/lJC9Zfh0FlPVkkvDnZVNsPEIEsg0Ta5oDlz1jIZmSB/Oxu2qRAyo4jXIsWSmDMdQdpNrwkyKbLfl/CT7PWe23RAdF8oQf5XqnSbKoapQali8MH4+HPOR8r13/k+cZv9eKqUvknmxZPiyJbp4oFzqrWDJSUqwTGQLEdbp76Hjrkuxu3Pa/I4jQSt3RRRbAUrZeV1Z79cLKg+22SvrhUKKzwxeEMcgp4rQzrMXhTL+wE+6sBczuguHmPtWA5w/NsUlevRaLbEionbyXYN";
    private static final double[][] TOP_GRIPPER_POSITIONS = {
            {0.5, 0.5}, //open {left, right}
            {1.0, 0.0}, // close
            {0.2, 1.0} // grip
    };

    private static final double[][] BOTTOM_GRIPPER_POSITIOINS = {
            {0.5, 0.5},
            {0.1, 0.9},
            {0.9, 0.1}
    };

    public HolonomicDriver driver;
    public Gripper topGripper;
    public Gripper bottomGripper;
    private DcMotor leftLift, rightLift;
    private Servo leftJewelServo, rightJewelServo;
    private ModernRoboticsI2cColorSensor leftSideColor, rightSideColor;
//    private GyroHandler gyroHandler;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;

    public static Robot9853 build(OpMode opMode) {
        return new Robot9853(opMode.hardwareMap, opMode.telemetry);
    }

    public Robot9853(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry);
    }

    @Override
    public void init() {
        driver = new HolonomicDriver(
                getHardwareMap().dcMotor.get("FrontLeftDriveMotor"),
                getHardwareMap().dcMotor.get("FrontRightDriveMotor"),
                getHardwareMap().dcMotor.get("BackLeftDriveMotor"),
                getHardwareMap().dcMotor.get("BackRightDriveMotor"),
                log
        );

        leftLift = getHardwareMap().dcMotor.get("LeftLiftMotor");
        rightLift = getHardwareMap().dcMotor.get("RightLiftMotor");

        leftJewelServo = getHardwareMap().servo.get("LeftJewelServo");
        rightJewelServo = getHardwareMap().servo.get("RightJewelServo");

        leftSideColor = (ModernRoboticsI2cColorSensor) getHardwareMap().colorSensor.get("LeftJewelColor");
        leftSideColor.setI2cAddress(I2cAddr.create8bit(0x3c));
        leftSideColor.enableLed(true);
        rightSideColor = (ModernRoboticsI2cColorSensor) getHardwareMap().colorSensor.get("RightJewelColor");
        rightSideColor.setI2cAddress(I2cAddr.create8bit(0x3a));
        rightSideColor.enableLed(true);

//        gyroHandler = GyroHandler.build(this);
//        gyroHandler.setOrientation(GyroHandler.GyroOrientation.UPSIDE_DOWN);
//        gyroHandler.setTolerance(GYRO_TOERANCE, AngleUnit.DEGREES);

        topGripper = new Gripper(
                getHardwareMap().servo.get("TopLeftLiftServo"),
                getHardwareMap().servo.get("TopRightLiftServo"),
                this,
                TOP_GRIPPER_POSITIONS[0],
                TOP_GRIPPER_POSITIONS[1],
                TOP_GRIPPER_POSITIONS[2]
        );

        bottomGripper = new Gripper(
                getHardwareMap().servo.get("BottomLeftLiftServo"),
                getHardwareMap().servo.get("BottomRightLiftServo"),
                this,
                BOTTOM_GRIPPER_POSITIOINS[0],
                BOTTOM_GRIPPER_POSITIOINS[1],
                BOTTOM_GRIPPER_POSITIOINS[2]
        );

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
//        gyroHandler.init();
    }

    public void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(
                getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", getHardwareMap().appContext.getPackageName())
        );
        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
    }

    @Override
    public void start() {
        topGripper.open();
        bottomGripper.open();

        raiseLeftArm();
        raiseRightArm();

        if (vuforia != null) relicTrackables.activate();

//        while (! gyroHandler.isInitialized()) log.update();
    }

    @Override
    public void stop() {
        raiseRightArm();

        leftSideColor.enableLed(false);
        rightSideColor.enableLed(false);
        raiseLeftArm();

        super.stop();
    }

    public void setLiftPower(double power) {
        leftLift.setPower(power);
        rightLift.setPower(power);
    }

    public void dropLeftArm() {
        leftJewelServo.setPosition(LEFT_ARM_POSITIONS[1]);
    }

    public void raiseLeftArm() {
        leftJewelServo.setPosition(LEFT_ARM_POSITIONS[0]);
    }

    public void dropRightArm() {
        rightJewelServo.setPosition(RIGHT_ARM_POSITIONS[0]);
    }

    public void raiseRightArm() {
        rightJewelServo.setPosition(RIGHT_ARM_POSITIONS[1]);
    }

    public int getLeftColor() {
        return leftSideColor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
    }

    public int getRightColor() {
        return rightSideColor.readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER);
    }

    public boolean isRed(int color) {return color > 9 && color < 13;}
    public boolean isBlue(int color) {return color < 4 && color > 1;}

    public RelicRecoveryVuMark getVuMark() {
        if (vuforia != null) return RelicRecoveryVuMark.from(relicTemplate);
        return null;
    }
//    /**
//     * Rotates the robot by the given angle.
//     * @param angle     the angle to rotate in radians
//     */
//    public void rotate(double angle) {
//        rotate(angle, AngleUnit.RADIANS, null);
//    }
//
//    /**
//     * Rotates the robot by the given angle and calls the callback when finished
//     * @param angle     the angle to rotate in radians
//     * @param callback  the callback
//     */
//    public void rotate(double angle, Runnable callback) {
//        rotate(angle, AngleUnit.RADIANS, callback);
//    }
//
//    /**
//     * Rotates the robot by the given angle.
//     * @param angle     the angle to rotate
//     * @param angleUnit the units of measure to use for the angles
//     */
//    public void rotate(double angle, @NonNull AngleUnit angleUnit) {
//        rotate(angle, angleUnit, null);
//    }
//
//    /**
//     * Rotates the robot by the given angle and calls the callback when finished
//     * @param angle     the angle to rotate
//     * @param angleUnit the units of measure to use for the angles
//     * @param callback  the callback
//     */
//    public void rotate(double angle, @NonNull AngleUnit angleUnit, Runnable callback) {
//        final double targetD = (angleUnit.toDegrees(angle) + gyroHandler.getRelativeHeading(AngleUnit.DEGREES)) % 360;
//        final double target = angleUnit.fromDegrees(targetD);
//        log.debug(targetD + " " + target);
//        final String unitName = angleUnit == AngleUnit.DEGREES ? "degrees" : "radians";
//
//        log.debugf("Rotating by %.2f %s to a heading of %.2f", angle, unitName, target);
//
//        gyroHandler.untilAtHeading(targetD, GYRO_TOERANCE, AngleUnit.DEGREES, dis -> {
//            log.debugf("%.2f %s away from the target heading of %.2f", angleUnit.fromDegrees(dis), unitName, target);
//            driver.rotate(Range.clip(dis * GYRO_SCALE_FACTOR / 180, 1, -1));
//            log.update();
//        }, () -> {
//            log.debug("Finished rotating");
//            driver.stop();
//
//            if (callback != null) callback.run();
//        });
//    }
//
//    /**
//     * Rotates the robot by the given angle synchronously (blocks thread)
//     * @param angle                 the angle to rotate in radians
//     * @throws InterruptedException thrown if the thread is interrupted while rotating
//     */
//    public void rotateSync(double angle) throws InterruptedException {
//        rotateSync(angle, AngleUnit.RADIANS);
//    }
//
//    /**
//     * Rotates the robot by the given angle synchronously (blocks thread)
//     * @param angle                 the angle to rotate
//     * @param angleUnit             the units of measure to use for the angles
//     * @throws InterruptedException thrown if the thread is interrupted while rotating
//     */
//    public void rotateSync(double angle, @NonNull AngleUnit angleUnit) throws InterruptedException {
//        final double targetD = (angleUnit.toDegrees(angle) + gyroHandler.getRelativeHeading(AngleUnit.DEGREES)) % 360;
//        final double target = angleUnit.fromDegrees(targetD);
//        final String unitName = angleUnit == AngleUnit.DEGREES ? "degrees" : "radians";
//
//        log.debugf("Rotating by %.2f %s to a heading of %.2f", angle, unitName, target);
//
//        gyroHandler.untilAtHeadingSync(targetD, GYRO_TOERANCE, AngleUnit.DEGREES, dis -> {
//            log.debugf("%.2f %s away from the target heading of %.2f", angleUnit.fromDegrees(dis), unitName, target);
//            driver.rotate(Range.clip(dis * GYRO_SCALE_FACTOR / -180, 1, -1));
//            log.update();
//        });
//
//        log.debug("Finished rotating");
//        driver.stop();
//    }
}
