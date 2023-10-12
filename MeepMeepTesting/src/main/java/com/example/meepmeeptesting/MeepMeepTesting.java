package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDimensions(15.75, 15.35)
                .setStartPose(new Pose2d(-63.88, -36.71, Math.toRadians(0.00)))
                .setConstraints(59.98047895234888, 59.98047895234888, Math.toRadians(234.7068470306528), Math.toRadians(234.7068470306528), 10.8)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-64.07, -40.29, Math.toRadians(0.00)))
                                .splineTo(new Vector2d(-11.42, -58.41), Math.toRadians(270.00))
                                .lineToConstantHeading(new Vector2d(-11.80, 34.07))
                                .splineTo(new Vector2d(-62.37, 12.36), Math.toRadians(180.00))
                                .waitSeconds(2)
                                .lineToConstantHeading(new Vector2d(-62.37, -40.29))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}