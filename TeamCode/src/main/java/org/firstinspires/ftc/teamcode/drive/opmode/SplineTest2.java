package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

import kotlin.Unit;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-40.0, -63.0, Math.toRadians(0.0)));
        Servo arm = hardwareMap.servo.get("leftArm");
        Servo flap = hardwareMap.servo.get("leftFlap");
        Servo left = hardwareMap.servo.get("blhook");
        Servo right = hardwareMap.servo.get("brhook");

        waitForStart();

        if (isStopRequested()) return;

        flap.setPosition(0);
        Trajectory driveToFirstBlock = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(-22.0, -32.0)).build(); // pick up first block

        drive.followTrajectorySync(driveToFirstBlock);
        arm.setPosition(0);
        sleep(250);
        flap.setPosition(1);
        sleep(350);
        arm.setPosition(1);
        sleep(100);

        Trajectory dropOffFirstBlock = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0.0, -36.0))
                .splineTo(new Pose2d(55.0, -30.0)) // drop off first block
                .build();

        drive.followTrajectorySync(dropOffFirstBlock);
        flap.setPosition(0);
        arm.setPosition(0);
        sleep(400);

        flap.setPosition(1);
        arm.setPosition(1);

        Trajectory driveToSecondBlock = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, -36.0))
                .lineTo(new Vector2d(-46.0, -32.0)) // pick up second block
                .build();

        drive.followTrajectorySync(driveToSecondBlock);
        flap.setPosition(0);
        arm.setPosition(0);
        sleep(450);
        flap.setPosition(1);
        sleep(350);
        arm.setPosition(1);
        sleep(100);

        Trajectory dropOffSecondBlock = drive.trajectoryBuilder()
                .splineTo(new Pose2d(0.0, -36.0))
                .splineTo(new Pose2d(50.0, -30.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffSecondBlock);
        flap.setPosition(0);
        arm.setPosition(0);
        sleep(400);

        flap.setPosition(1);
        arm.setPosition(1);

        Trajectory backupAndPrepForTurn = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .strafeTo(new Vector2d(42, -36.0)) // turn
                .lineTo(new Vector2d(42, -36), new ConstantInterpolator(Math.toRadians(-90)))
                .build();

        drive.followTrajectorySync(backupAndPrepForTurn);

        drive.turnSync(Math.toRadians(-90));

        Trajectory backupAndGrabPlate = drive.trajectoryBuilder()
                .reverse() // drive backwards
                .lineTo(new Vector2d(42, -30.0)) // backup
                .build();

        drive.followTrajectorySync(backupAndGrabPlate);

        left.setPosition(.3);
        right.setPosition(.6);
        sleep(600);

        Trajectory pullAndTurn = drive.trajectoryBuilder()
                .lineTo(new Vector2d(42.0, -53.0)) // drag forward and turn
                .build();

        drive.followTrajectorySync(pullAndTurn);

        drive.turnWithTimeoutSync(Math.toRadians(-90), 3.5);

        left.setPosition(.7);
        right.setPosition(.17);
        sleep(600);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0.0, -34)) // drive to bridge
                .build();
        drive.followTrajectorySync(driveToBridge);
    }
}
