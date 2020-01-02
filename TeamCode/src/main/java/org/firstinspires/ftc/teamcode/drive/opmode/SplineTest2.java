package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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

        Trajectory trajectory = drive.trajectoryBuilder()
                .strafeTo(new Vector2d(-22.0, -32.0)) // pick up first block
                .addMarker(() -> { sleep(2000); return Unit.INSTANCE;})
                .lineTo(new Vector2d(0.0, -40.0))
                .splineTo(new Pose2d(60.0, -30.0)) // drop off first block
                .addMarker(() -> { sleep(2000); return Unit.INSTANCE;})
                .reverse() // drive backwards
                .splineTo(new Pose2d(0.0, -40.0))
                .lineTo(new Vector2d(-46.0, -32.0)) // pick up second block
                .addMarker(() -> { sleep(2000); return Unit.INSTANCE;})
                .reverse() // drive forwards
                .splineTo(new Pose2d(0.0, -40.0))
                .splineTo(new Pose2d(50.0, -30.0)) // drop off second block
                .addMarker(() -> { sleep(2000); return Unit.INSTANCE;})
                .reverse() // drive backwards
                .lineTo(new Vector2d(42.0, -34.0), new ConstantInterpolator(Math.toRadians(0.0))) // turn
                .lineTo(new Vector2d(42.0, -30.0), new ConstantInterpolator(Math.toRadians(-90.0))) // backup
                .addMarker(() -> { sleep(2000); return Unit.INSTANCE;})
                .lineTo(new Vector2d(42.0, -54.0), new ConstantInterpolator(Math.toRadians(-180.0))) // drag forward and turn
                .lineTo(new Vector2d(0.0, -40.0), new ConstantInterpolator(Math.toRadians(-180.0)))
                .addMarker(() -> { sleep(2000); return Unit.INSTANCE;})
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectorySync(trajectory);

    }
}
