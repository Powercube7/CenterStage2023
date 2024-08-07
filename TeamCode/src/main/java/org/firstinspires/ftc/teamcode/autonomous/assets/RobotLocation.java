package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

public enum RobotLocation {
    RED_SHORT(1, 1), BLUE_SHORT(-1, 1), RED_LONG(1, -1), BLUE_LONG(-1, -1);

    public final int color, side;

    RobotLocation(int color, int side) {
        this.color = color;
        this.side = side;
    }

    public Pose getStartingPose() {
        double x, y = -63.75, theta = 90.00;
        x = side == 1 ? 16.75 : -40.25;

        return new Pose(x, color * y, Math.toRadians(color * theta));
    }
}
