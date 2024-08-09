package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public enum AllianceColor {
    RED, BLUE;

    // TODO: Find robot's starting pose
    private final Pose startingPose = new Pose();

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
