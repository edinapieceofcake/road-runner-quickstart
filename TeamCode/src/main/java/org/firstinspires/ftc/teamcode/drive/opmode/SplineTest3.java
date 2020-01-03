package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.SplineInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        Servo arm = hardwareMap.servo.get("leftArm");
        Servo flap = hardwareMap.servo.get("leftFlap");
        Servo left = hardwareMap.servo.get("blhook");
        Servo right = hardwareMap.servo.get("brhook");

        left.setPosition(.7);
        right.setPosition(.17);

        waitForStart();

        if (isStopRequested()) return;
/*
        Trajectory dropOffSecondBlock = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(20, -30.0)) // drop off second block
                .build();

        drive.followTrajectorySync(dropOffSecondBlock);
        sleep(2000);
*/
        drive.setPoseEstimate(new Pose2d(50, -30.0, Math.toRadians(0.0)));
        Trajectory backupAndPrepForTurn = drive.trajectoryBuilder()
                .reverse() // drive backwards
//                .splineTo(new Pose2d(42, -36), new SplineInterpolator(Math.toRadians(0), Math.toRadians(-90)))
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
        sleep(800);

        Trajectory pullAndTurn = drive.trajectoryBuilder()
                .lineTo(new Vector2d(42.0, -53.0)) // drag forward and turn
                .build();

        drive.followTrajectorySync(pullAndTurn);

        drive.turnWithTimeoutSync(Math.toRadians(-90), 4);

        left.setPosition(.7);
        right.setPosition(.17);
        sleep(800);

        Trajectory driveToBridge = drive.trajectoryBuilder()
                .lineTo(new Vector2d(0.0, -34)) // drive to bridge
                .build();
        drive.followTrajectorySync(driveToBridge);
    }
}
