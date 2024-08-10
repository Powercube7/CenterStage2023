package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous
public class TestAuto extends CommandOpMode {
    public static AllianceColor allianceColor = AllianceColor.BLUE;

    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(allianceColor.getStartingPose());

        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);

        PathChain purplePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(allianceColor.getStartingPose()),
                        new Point(35.25, 80, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(0, Math.toRadians(-45), 0.9)
                .build();

        PathChain yellowPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(35.25, 80, Point.CARTESIAN),
                        new Point(42.25, 120, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90), 0.9)
                .build();

        PathChain stackPath = follower.pathBuilder()
                // goes to closest stack (middle truss)
                .addPath(new BezierCurve(
                        new Point(42.25, 120, Point.CARTESIAN),
                        new Point(33, 68.5, Point.CARTESIAN),
                        new Point(36, 18, Point.CARTESIAN)
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
                        new FollowPathCommand(follower, purplePath),

                        // Place purple
                        new InstantCommand(intake::toggleClamp)
                                .andThen(
                                        new InstantCommand(outtake::toggleBlockers), // prepare for yellow
                                        new WaitCommand(200)
                                ),

                        // Head for yellow
                        new FollowPathCommand(follower, yellowPath)
                                .alongWith(new InstantCommand(outtake::toggleSpike)),

                        // Place yellow
                        new InstantCommand(() -> {
                            outtake.toggleBlockers();
                            outtake.toggleBlockers();
                        }).andThen(
                                new WaitCommand(250),
                                new InstantCommand(outtake::toggleSpike)
                        ),

                        new FollowPathCommand(follower, stackPath)
                )
        );
    }
}
