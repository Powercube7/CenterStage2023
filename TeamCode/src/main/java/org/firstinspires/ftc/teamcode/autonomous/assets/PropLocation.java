package org.firstinspires.ftc.teamcode.autonomous.assets;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public enum PropLocation {
    // TODO: Find purple and yellow pixel poses
    TRUSS_SIDE(new Pose(), new Pose()),
    MIDDLE(new Pose(34, 84, Math.toRadians(0)), new Pose(35, 120, Math.toRadians(-90))),
    BACKDROP_SIDE(new Pose(33, 80, Math.toRadians(-45)), new Pose(42.25, 120, Math.toRadians(-90)));

    private final Pose purplePose, yellowPose;

    PropLocation(Pose purplePose, Pose yellowPose) {
        this.purplePose = purplePose;
        this.yellowPose = yellowPose;
    }

    public Path getPurplePath(AllianceColor allianceColor) {
        Pose start = allianceColor.getStartingPose();
        Pose end = allianceColor.convertPose(purplePose);

        Path path = new Path(new BezierLine(
                new Point(start),
                new Point(end)
        ));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), 0.9);

        return path;
    }

    public Path getYellowPath(AllianceColor allianceColor) {
        Pose start = allianceColor.convertPose(purplePose);
        Pose end = allianceColor.convertPose(yellowPose);

        Path path = new Path(new BezierLine(
                new Point(start),
                new Point(end)
        ));
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading(), 0.9);

        return path;
    }

    @NonNull
    @Override
    public String toString() {
        String name = this.name();
        return name.charAt(0) + name.substring(1).toLowerCase();
    }
}
