package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

public class DashboardPose {

    public double x, y, theta;
    public AngleUnit angleUnit = AngleUnit.DEGREES;

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
        theta = angleUnit.fromRadians(pose.getHeading());
    }

    public Pose2d toPose2d() {
        return new Pose2d(x, y, angleUnit.toRadians(theta));
    }

    public Vector2d vec() {
        return new Vector2d(x, y);
    }

    public double heading(AngleUnit unit) {
        return unit.fromUnit(angleUnit, theta);
    }

    public DashboardPose mirrored() {
        DashboardPose newPose = new DashboardPose(x, -y, -theta);
        newPose.angleUnit = angleUnit;

        return newPose;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "x: %.2f, y: %.2f, heading: %.2f", x, y, theta);
    }
}
