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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.common.robot.Robot;
import org.chathamrobotics.common.systems.Gripper;
import org.chathamrobotics.common.systems.GyroHandler;
import org.chathamrobotics.common.systems.HolonomicDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team9853.systems.JewelDisplacer;

/**
 * Team 9853's robot
 */
@SuppressWarnings({"unused", "WeakerAccess"})
public class Robot9853 extends Robot {

    private static final int GYRO_TOERANCE = 15;
    private static final double GYRO_TOERANCE_RAD = Math.toRadians(25);
    private static final int GYRO_SCALE_FACTOR = 2;
    private static final int[] LEFT_ARM_POSITIONS = {0, 1}; //up, down
    private static final int[] RIGHT_ARM_POSITIONS = {0, 1}; // up, down

    public HolonomicDriver driver;
    public Gripper topGripper;
    public Gripper bottomGripper;
    public JewelDisplacer jewelDisplacer;
    private DcMotor leftLift, rightLift;
    private Servo leftJewelServo, rightJewelServo;
    private ModernRoboticsI2cColorSensor leftSideColor, rightSideColor;
    private GyroHandler gyroHandler;

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
        rightSideColor = (ModernRoboticsI2cColorSensor) getHardwareMap().colorSensor.get("RightJewelColor");

        gyroHandler = GyroHandler.build(this);
        gyroHandler.setOrientation(GyroHandler.GyroOrientation.UPSIDE_DOWN);
        gyroHandler.setTolerance(GYRO_TOERANCE, AngleUnit.DEGREES);

        topGripper = new Gripper(
                getHardwareMap().servo.get("TopLeftLiftServo"),
                getHardwareMap().servo.get("TopRightLiftServo"),
                this
        );

        bottomGripper = new Gripper(
                getHardwareMap().servo.get("BottomLeftLiftServo"),
                getHardwareMap().servo.get("BottomRightLiftServo"),
                this
        );

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        jewelDisplacer.raise();
        gyroHandler.init();
    }

    @Override
    public void start() {
        topGripper.open();
        bottomGripper.open();

        raiseLeftArm();
        raiseRightArm();

        while (! gyroHandler.isInitialized()) log.update();
    }

    @Override
    public void stop() {
        jewelDisplacer.raise();

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
        rightJewelServo.setPosition(RIGHT_ARM_POSITIONS[1]);
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

    /**
     * Rotates the robot by the given angle.
     * @param angle     the angle to rotate in radians
     */
    public void rotate(double angle) {
        rotate(angle, AngleUnit.RADIANS, null);
    }

    /**
     * Rotates the robot by the given angle and calls the callback when finished
     * @param angle     the angle to rotate in radians
     * @param callback  the callback
     */
    public void rotate(double angle, Runnable callback) {
        rotate(angle, AngleUnit.RADIANS, callback);
    }

    /**
     * Rotates the robot by the given angle.
     * @param angle     the angle to rotate
     * @param angleUnit the units of measure to use for the angles
     */
    public void rotate(double angle, @NonNull AngleUnit angleUnit) {
        rotate(angle, angleUnit, null);
    }

    /**
     * Rotates the robot by the given angle and calls the callback when finished
     * @param angle     the angle to rotate
     * @param angleUnit the units of measure to use for the angles
     * @param callback  the callback
     */
    public void rotate(double angle, @NonNull AngleUnit angleUnit, Runnable callback) {
        final double targetD = (angleUnit.toDegrees(angle) + gyroHandler.getRelativeHeading(AngleUnit.DEGREES)) % 360;
        final double target = angleUnit.fromDegrees(targetD);
        log.debug(targetD + " " + target);
        final String unitName = angleUnit == AngleUnit.DEGREES ? "degrees" : "radians";

        log.debugf("Rotating by %.2f %s to a heading of %.2f", angle, unitName, target);

        gyroHandler.untilAtHeading(targetD, GYRO_TOERANCE, AngleUnit.DEGREES, dis -> {
            log.debugf("%.2f %s away from the target heading of %.2f", angleUnit.fromDegrees(dis), unitName, target);
            driver.rotate(Range.clip(dis * GYRO_SCALE_FACTOR / 180, 1, -1));
            log.update();
        }, () -> {
            log.debug("Finished rotating");
            driver.stop();

            if (callback != null) callback.run();
        });
    }

    /**
     * Rotates the robot by the given angle synchronously (blocks thread)
     * @param angle                 the angle to rotate in radians
     * @throws InterruptedException thrown if the thread is interrupted while rotating
     */
    public void rotateSync(double angle) throws InterruptedException {
        rotateSync(angle, AngleUnit.RADIANS);
    }

    /**
     * Rotates the robot by the given angle synchronously (blocks thread)
     * @param angle                 the angle to rotate
     * @param angleUnit             the units of measure to use for the angles
     * @throws InterruptedException thrown if the thread is interrupted while rotating
     */
    public void rotateSync(double angle, @NonNull AngleUnit angleUnit) throws InterruptedException {
        final double targetD = (angleUnit.toDegrees(angle) + gyroHandler.getRelativeHeading(AngleUnit.DEGREES)) % 360;
        final double target = angleUnit.fromDegrees(targetD);
        final String unitName = angleUnit == AngleUnit.DEGREES ? "degrees" : "radians";

        log.debugf("Rotating by %.2f %s to a heading of %.2f", angle, unitName, target);

        gyroHandler.untilAtHeadingSync(targetD, GYRO_TOERANCE, AngleUnit.DEGREES, dis -> {
            log.debugf("%.2f %s away from the target heading of %.2f", angleUnit.fromDegrees(dis), unitName, target);
            driver.rotate(Range.clip(dis * GYRO_SCALE_FACTOR / -180, 1, -1));
            log.update();
        });

        log.debug("Finished rotating");
        driver.stop();
    }
}
