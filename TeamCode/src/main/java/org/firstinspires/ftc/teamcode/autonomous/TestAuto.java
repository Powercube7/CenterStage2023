package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous
public class TestAuto extends CommandOpMode {

    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(AllianceColor.BLUE.getStartingPose());

        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);

        PathChain purplePath = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Point(7.5787401574803149606299212598425, 87.007874015748031496062992125984, Point.CARTESIAN),
                                new Point(35.117493472584854, 144 - 63.96605744125327, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(0, Math.toRadians(-45), 0.9)
                .build();

        PathChain yellowPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(35.117493472584854, 144 - 63.96605744125327, Point.CARTESIAN),
                        new Point(42.297650130548305, 144 - 24.39686684073107, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90), 0.9)
                .build();

        PathChain stackPath = follower.pathBuilder()
                // goes to closest stack (middle truss)
                .addPath(new BezierCurve(
                        new Point(42.297650130548305, 144 - 24.39686684073107, Point.CARTESIAN),
                        new Point(32.146214099216714, 144 - 75.47780678851176, Point.CARTESIAN),
                        new Point(35.90600522193211, 144 - 125.49869451697128, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        // Start purple following
                        new InstantCommand(() -> {
                            intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                            intake.adjustLiftPosition(10.0);
                        }),
                        new InstantCommand(() -> follower.followPath(purplePath, true))
                                .andThen(new WaitUntilCommand(() -> !follower.isBusy())),

                        // Place purple
                        new InstantCommand(intake::toggleClamp)
                                .andThen(
                                        new InstantCommand(outtake::toggleBlockers), // prepare for yellow
                                        new WaitCommand(200)
                                ),

                        // Head for yellow
                        new InstantCommand(() -> follower.followPath(yellowPath, true))
                                .alongWith(
                                        new InstantCommand(outtake::toggleSpike),
                                        new WaitUntilCommand(() -> !follower.isBusy())
                                ),

                        // Place yellow
                        new InstantCommand(() -> {
                            outtake.toggleBlockers();
                            outtake.toggleBlockers();
                        }).andThen(
                                new WaitCommand(250),
                                new InstantCommand(outtake::toggleSpike)
                        ),

                        new InstantCommand(() -> follower.followPath(stackPath, true))
                )
        );
    }
}
