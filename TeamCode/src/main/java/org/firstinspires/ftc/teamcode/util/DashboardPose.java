package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.Locale;

public class DashboardPose {

    public double x, y, theta;

    public DashboardPose() {
    }

    public DashboardPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public DashboardPose(Pose2d pose) {
        x = pose.getX();
        y = pose.getY();
        theta = pose.getHeading();
    }

    public static DashboardPose polar(double r, double theta) {
        Vector2d vec = Vector2d.polar(r, theta);
        return new DashboardPose(vec.getX(), vec.getY(), theta);
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, theta);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "x: %.2f, y: %.2f, heading: %.2f", x, y, theta);
    }
}
