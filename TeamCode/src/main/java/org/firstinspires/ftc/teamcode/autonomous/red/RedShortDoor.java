package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.PathGenerator;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocations;
import org.firstinspires.ftc.teamcode.autonomous.assets.StartingPosition;
import org.firstinspires.ftc.teamcode.commands.AwaitPixelDetectionCommand;
import org.firstinspires.ftc.teamcode.commands.RunByCaseCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.DashboardPose;

import java.util.Locale;

@Config
@Autonomous(name = "Red Short (Stage Door)")
public class RedShortDoor extends CommandOpMode {

    public static DashboardPose STACK_POSE = new DashboardPose(-57.25, -13.00, 180);
    private PropLocations location = PropLocations.LEFT;
    private SampleMecanumDrive drive;

    @Override
    public void initialize() {
        TensorflowSubsystem tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                "red_prop.tflite", "Red Prop");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Loading trajectories...");
        telemetry.update();

        drive = new SampleMecanumDrive(hardwareMap);
        PathGenerator generator = new PathGenerator(drive);
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        colorSensor.initialize();

        OdometrySubsystem odometry = new OdometrySubsystem(this);
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);

        generator.setStartingLocation(AllianceColor.RED, StartingPosition.BACKDROP);
        tensorflow.setMinConfidence(0.8);

        Trajectory middlePurple = drive.trajectoryBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(15, -38), Math.toRadians(90.00))
                .build();
        Trajectory leftPurple = drive.trajectoryBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(.5, -33)
                        .minus(Vector2d.polar(12, Math.toRadians(135))), Math.toRadians(135.00))
                .build();
        Trajectory rightPurple = drive.trajectoryBuilder(generator.getStartingPose())
                .splineTo(new Vector2d(23.5, -32)
                        .minus(Vector2d.polar(13, Math.toRadians(60))), Math.toRadians(60.00))
                .build();

        Trajectory leftYellow = drive.trajectoryBuilder(leftPurple.end())
                .lineToLinearHeading(new Pose2d(48.75, -29.50, Math.toRadians(180.00)))
                .build();
        Trajectory middleYellow = drive.trajectoryBuilder(middlePurple.end())
                .lineToLinearHeading(new Pose2d(48.75, -35.50, Math.toRadians(180.00)))
                .build();
        Trajectory rightYellow = drive.trajectoryBuilder(rightPurple.end())
                .lineToLinearHeading(new Pose2d(48.75, -42.50, Math.toRadians(180.00)))
                .build();

        TrajectorySequence stackLeft = drive.trajectorySequenceBuilder(leftYellow.end(), 50)
                .splineTo(new Vector2d(18, STACK_POSE.y), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(STACK_POSE.toPose2d())
                .build();
        TrajectorySequence stackMid = drive.trajectorySequenceBuilder(middleYellow.end(), 50)
                .splineTo(new Vector2d(18, STACK_POSE.y), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(STACK_POSE.toPose2d())
                .build();
        TrajectorySequence stackRight = drive.trajectorySequenceBuilder(rightYellow.end(), 50)
                .splineTo(new Vector2d(18, STACK_POSE.y), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(STACK_POSE.toPose2d())
                .build();

        TrajectorySequence backdropLeft = drive.trajectorySequenceBuilder(stackLeft.end())
                .setReversed(true)
                .splineTo(new Pose2d(24, STACK_POSE.y, Math.toRadians(0.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(50.00, -30.50), Math.toRadians(0.00))
                .build();
        TrajectorySequence backdropMiddle = drive.trajectorySequenceBuilder(stackMid.end())
                .setReversed(true)
                .splineTo(new Pose2d(24, STACK_POSE.y, Math.toRadians(0.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(50.00, -30.50), Math.toRadians(0.00))
                .build();
        TrajectorySequence backdropRight = drive.trajectorySequenceBuilder(stackRight.end())
                .setReversed(true)
                .splineTo(new Pose2d(24, STACK_POSE.y, Math.toRadians(0.00)))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(50.00, -30.50), Math.toRadians(0.00))
                .build();

        TrajectorySequence stackTwoLeft = drive.trajectorySequenceBuilder(backdropLeft.end(), 50)
                .splineTo(new Vector2d(18, STACK_POSE.y), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(STACK_POSE.toPose2d())
                .build();

        TrajectorySequence stackTwoMid = drive.trajectorySequenceBuilder(backdropMiddle.end(), 50)
                .splineTo(new Vector2d(18, STACK_POSE.y), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(STACK_POSE.toPose2d())
                .build();
        TrajectorySequence stackTwoRight = drive.trajectorySequenceBuilder(backdropRight.end(), 50)
                .splineTo(new Vector2d(18, STACK_POSE.y), Math.toRadians(180))
                .waitSeconds(.25)
                .lineToLinearHeading(STACK_POSE.toPose2d())
                .build();

        while (!isStarted()) {
            if (isStopRequested())
                return;

            Recognition bestDetection = tensorflow.getBestDetection();
            location = PropLocations.LEFT;

            if (bestDetection != null) {
                double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2.0;
                location = x < (bestDetection.getImageWidth() / 2.0) ? PropLocations.MIDDLE : PropLocations.RIGHT;
            }

            telemetry.addData("FPS", tensorflow.portal.getFps());
            telemetry.addData("Current Location", location.toString());
            telemetry.addData("Confidence", String.format(Locale.US, "%.2f%%", bestDetection != null ? bestDetection.getConfidence() * 100 : 0));
            telemetry.update();
        }

        tensorflow.shutdown();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> {
                    intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                    intake.adjustLiftPosition(10.0);
                }),
                new RunByCaseCommand(location.toString(), drive, leftPurple, middlePurple, rightPurple, true),
                new InstantCommand(intake::toggleLiftLocation).andThen(
                        new InstantCommand(() -> {
                            intake.setClampPosition(25);
                            intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);

                            outtake.toggleBlockers();
                            outtake.setSpikePosition(.875);
                        })
                ),
                new RunByCaseCommand(location.toString(), drive, leftYellow, middleYellow, rightYellow, true),
                new InstantCommand(outtake::toggleBlockers).andThen(
                        new WaitCommand(300),
                        new InstantCommand(outtake::toggleBlockers),
                        new WaitCommand(300),
                        new InstantCommand(outtake::toggleSpike)
                ),
                new ParallelRaceGroup(
                        new RunByCaseCommand(location.toString(), drive, stackLeft, stackMid, stackRight, false),
                        new AwaitPixelDetectionCommand(colorSensor,
                                () -> {
                                    drive.breakFollowing();
                                    intake.toggleClamp();
                                }, intake::toggleClamp
                        )
                ).andThen(new WaitCommand(250)),
                new ParallelCommandGroup(
                        new RunByCaseCommand(location.toString(), drive, backdropLeft, backdropMiddle, backdropRight, false),
                        new WaitCommand(700)
                                .andThen(new InstantCommand(intake::toggleClamp)),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {
                                            intake.setClampPosition(25);
                                            intake.adjustLiftPosition(10.0);
                                            outtake.toggleBlockers();
                                            outtake.toggleSpike();
                                        }),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setSlidesTicks(200))
                                )
                ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300)
                        ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new InstantCommand(() -> outtake.setSlidesPosition(0))
                        ),
                new ParallelRaceGroup(
                        new RunByCaseCommand(location.toString(), drive, stackTwoLeft, stackTwoMid, stackTwoRight, false),
                        new AwaitPixelDetectionCommand(colorSensor,
                                () -> {
                                    drive.breakFollowing();
                                    intake.toggleClamp();
                                }, intake::toggleClamp
                        )
                ).andThen(new WaitCommand(250)),
                new ParallelCommandGroup(
                        new RunByCaseCommand(location.toString(), drive, backdropLeft, backdropMiddle, backdropRight, false),
                        new WaitCommand(700)
                                .andThen(new InstantCommand(intake::toggleClamp)),
                        new WaitUntilCommand(() -> drive.getPoseEstimate().getX() > 0)
                                .andThen(
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> {
                                            intake.setClampPosition(25);
                                            outtake.toggleBlockers();
                                            outtake.toggleSpike();
                                        }),
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setSlidesTicks(300))
                                )
                ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new WaitCommand(300)
                        ),
                new InstantCommand(outtake::toggleBlockers)
                        .andThen(
                                new WaitCommand(300),
                                new InstantCommand(outtake::toggleSpike),
                                new InstantCommand(() -> outtake.setSlidesPosition(0))
                        )
        ));
    }

    @Override
    public void run() {
        super.run();

        if (!drive.isBusy())
            drive.updatePoseEstimate();
    }
}
