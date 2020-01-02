package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTestWithPlate extends LinearOpMode {
    public static double ANGLE = 90; // deg
    public static double TIMEOUT = 2;
    public static double SLEEPTIME = 1000;

    private Servo _left;
    private Servo _right;

    @Override
    public void runOpMode() {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        _left = hardwareMap.servo.get("blhook");
        _right = hardwareMap.servo.get("brhook");

        waitForStart();

        if (isStopRequested()) return;

        dropHooks();
        sleep((long)SLEEPTIME);
        drive.turnWithTimeoutSync(Math.toRadians(ANGLE), TIMEOUT);
        liftHooks();
    }


    private void dropHooks() {
        _left.setPosition(.3);
        _right.setPosition(.6);
    }


    private void liftHooks() {
        _left.setPosition(.7);
        _right.setPosition(.17);
    }
}
