package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        OdometrySubsystem odometry = new OdometrySubsystem(this);
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d meanSquaredError = new Pose2d();

        waitForStart();
        if (isStopRequested()) return;

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .setTangent(Math.PI / 2.0)
                .splineToConstantHeading(new Vector2d(30, 30), 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineTo(new Vector2d(), Math.PI)
                .build();

        drive.followTrajectorySequenceAsync(traj);
        while (opModeIsActive()) {
            if (drive.isBusy()) {
                drive.update();

                Pose2d error = drive.getLastError();
                meanSquaredError = meanSquaredError.plus(new Pose2d(
                        error.getX() * error.getX(),
                        error.getY() * error.getY(),
                        Math.toDegrees(Angle.normDelta(error.getHeading())) * Math.toDegrees(Angle.normDelta(error.getHeading()))
                ));
            }

            telemetry.addData("Mean Squared Error", meanSquaredError.toString());
            telemetry.update();
        }
    }
}
