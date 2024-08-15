package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class StackGenerator {
    private final Point stackLocation, backdropLocation;
    private AllianceColor allianceColor = AllianceColor.BLUE;
    private Follower follower;
    public StackGenerator(Point stackLocation, Point backdropLocation) {
        this.stackLocation = stackLocation;
        this.backdropLocation = backdropLocation;
    }

    public StackGenerator setAlliance(AllianceColor alliance) {
        allianceColor = alliance;
        return this;
    }

    public StackGenerator setFollower(Follower follower) {
        this.follower = follower;
        return this;
    }

    public PathChain getStackPath(Point startPoint, Route route) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = new PathBuilder();

        if (route == Route.MIDDLE_TRUSS)
            builder.addPath(new BezierCurve(
                            startPoint,
                            allianceColor.convertPoint(new Point(36, 96, Point.CARTESIAN)),
                            allianceColor.convertPoint(new Point(36, 48, Point.CARTESIAN)),
                            stackLocation
                    ))
                    .setConstantHeadingInterpolation(Math.toRadians(-90))
                    .setZeroPowerAccelerationMultiplier(4.0);

        else if (route == Route.WALL_TRUSS)
            builder.addPath(new BezierCurve(
                            startPoint,
                            allianceColor.convertPoint(new Point(6.5, 112.5, Point.CARTESIAN)),
                            allianceColor.convertPoint(new Point(12, 35.25, Point.CARTESIAN))
                    ))
                    .addParametricCallback(0.66, () -> follower.setMaxPower(0.66))
                    .setPathEndTValueConstraint(0.9) // faster end due to overshoot
                    .setTangentHeadingInterpolation()
                    .addPath(new BezierCurve(
                            allianceColor.convertPoint(new Point(12, 35.25, Point.CARTESIAN)),
                            allianceColor.convertPoint(new Point(36, 35, Point.CARTESIAN)),
                            stackLocation
                    ))
                    .addParametricCallback(0.99, () -> follower.setMaxPower(1.0))
                    .setConstantHeadingInterpolation(Math.toRadians(-90));

        return builder.build();
    }

    public PathChain getBackdropPath(Route route) throws IllegalStateException {
        if (follower == null)
            throw new IllegalStateException("The generator's follower wasn't set");

        PathBuilder builder = new PathBuilder();

        if (route == Route.MIDDLE_TRUSS)
            builder.addPath(new BezierLine(stackLocation, backdropLocation));

        else if (route == Route.WALL_TRUSS)
            builder.addPath(new BezierCurve(
                    stackLocation,
                    allianceColor.convertPoint(new Point(3.5, 11, Point.CARTESIAN)),
                    allianceColor.convertPoint(new Point(2, 100, Point.CARTESIAN)),
                    backdropLocation
            ));

        builder.addParametricCallback(0.1, () -> follower.setMaxPower(1.0))
                .setConstantHeadingInterpolation(Math.toRadians(-90));

        return builder.build();
    }

    public enum Route {
        WALL_TRUSS, MIDDLE_TRUSS
    }
}
