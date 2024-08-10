package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public enum AllianceColor {
    RED, BLUE;

    private final Pose startingPose = new Pose(7.575, 87, Math.toRadians(0));

    public Pose convertPose(Pose pose) {
        if (this == BLUE)
            return pose;
        else
            return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
    }

    public Pose getStartingPose() {
        return this.convertPose(startingPose);
    }
}
