package org.firstinspires.ftc.teamcode.autonomous.assets;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public enum PropLocation {
    // TODO: Find purple and yellow pixel poses
    LEFT(new Pose(), new Pose()), MIDDLE(new Pose(), new Pose()), RIGHT(new Pose(), new Pose());

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
        path.setLinearHeadingInterpolation(start.getHeading(), end.getHeading());

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
