package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    private static Vector2d mirrorVector(Vector2d vec) {
        return new Vector2d(vec.getX(), -vec.getY());
    }

    private static Pose2d mirrorPose(Pose2d pose) {
        return new Pose2d(mirrorVector(pose.vec()), -pose.getHeading());
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(640, 144);

        RoadRunnerBotEntity redBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(225), 8.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-58.00, -13.25, Math.toRadians(180.00)))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(51.50, -23.00, Math.toRadians(150.00)), Math.toRadians(-20))
                                .setTangent(Math.toRadians(160))
                                .splineToSplineHeading(new Pose2d(-58.00, -13.25, Math.toRadians(180.00)), Math.toRadians(180))
                                .build()
                );

        RoadRunnerBotEntity blueBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15.19)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(270), Math.toRadians(225), 8.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(52.50, -21.00, Math.toRadians(150)))
                                .setTangent(Math.toRadians(165))
                                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, Math.toRadians(270), 8.9))
                                .splineToSplineHeading(new Pose2d(-58.00, -13.00, Math.PI), Math.toRadians(180))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(52.50, -21.00, Math.toRadians(150)), Math.toRadians(-15))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(redBot)
//                .addEntity(blueBot)
                .start();
    }
}