package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * <p>This is an autonomous prototype to test the capabilities of the Pedro Pathing follower.</p>
 * <p>It doesn't have any callbacks or actions. It consists of 3 paths exclusively for testing purposes.</p>
 */
@Autonomous
public class TestAuto extends CommandOpMode {

    @Override
    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7.707571801566579, 144 - 54.26304686152363, Math.toRadians(0)));

        PathChain path = follower.pathBuilder()
                // deposits purple on the right
                .addPath(new BezierLine(
                        new Point(7.707571801566579, 144 - 54.26304686152363, Point.CARTESIAN),
                        new Point(38.003479322535206, 144 - 62.671303640049054, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                // goes to backdrop
                .addPath(new BezierLine(
                        new Point(38.003479322535206, 144 - 62.671303640049054, Point.CARTESIAN),
                        new Point(42.297650130548305, 144 - 20.39686684073107, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90), 0.9)
                // goes to closest stack (middle truss)
                .addPath(new BezierCurve(
                        new Point(42.297650130548305, 144 - 20.39686684073107, Point.CARTESIAN),
                        new Point(32.146214099216714, 144 - 75.47780678851176, Point.CARTESIAN),
                        new Point(35.90600522193211, 144 - 131.49869451697128, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        schedule(
                new RunCommand(follower::update),
                new InstantCommand(() -> follower.followPath(path, true))
        );
    }
}
