package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum RobotLocation {
    RED_SHORT(1, 1), BLUE_SHORT(-1, 1), RED_LONG(1, -1), BLUE_LONG(-1, -1);

    public final int color, side;

    RobotLocation(int color, int side) {
        this.color = color;
        this.side = side;
    }

    public Pose2d getStartingPose() {
        double x, y = -63.75, theta = 90.00;
        x = side == 1 ? 16.75 : -40.25;

        return new Pose2d(x, color * y, Math.toRadians(color * theta));
    }
}
