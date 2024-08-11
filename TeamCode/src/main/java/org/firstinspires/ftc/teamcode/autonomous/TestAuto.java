package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.SensorDetectionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Config
@Autonomous
public class TestAuto extends CommandOpMode {
    public static AllianceColor allianceColor = AllianceColor.BLUE;

    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(allianceColor.getStartingPose());

        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);
        RevColorSensorV3 clawSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        PathChain purplePath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(allianceColor.getStartingPose()),
                        new Point(34, 84, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        PathChain yellowPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(34, 84, Point.CARTESIAN),
                        new Point(35, 120, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90), 0.9)
                .build();

        PathChain stackPath = follower.pathBuilder()
                // goes to closest stack (middle truss)
//                .addPath(new BezierCurve(
//                        new Point(35, 120, Point.CARTESIAN),
//                        new Point(32, 115, Point.CARTESIAN),
//                        new Point(35, 13, Point.CARTESIAN)
//                ))
                .addPath(new BezierLine(
                        new Point(35, 120, Point.CARTESIAN),
                        new Point(35, 13, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
        PathChain backdropPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(35, 13, Point.CARTESIAN),
                        new Point(35, 120, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        schedule(
                new RunCommand(follower::update),
                new FixedSequentialCommandGroup(
                        // Start purple following
                        new InstantCommand(() -> {
                            intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                            intake.adjustLiftPosition(10.0);
                        }),
                        new FollowPathCommand(follower, purplePath),

                        // Place purple
                        new InstantCommand(intake::toggleClamp)
                                .andThen(
                                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.TWO_PIXELS)), // prepare for yellow
                                        new WaitCommand(200),
                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK))
                                ),

                        // Head for yellow
                        new FollowPathCommand(follower, yellowPath)
                                .alongWith(new InstantCommand(outtake::toggleSpike)),

                        // Place yellow
                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.ONE_PIXEL)).andThen(
                                new WaitCommand(150),
                                new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.FREE)),
                                new WaitCommand(300),
                                new InstantCommand(() -> {
                                    outtake.toggleSpike();
                                    intake.setClampPosition(25);
                                })
                        ),

                        new ParallelRaceGroup(
                                new FixedSequentialCommandGroup(
                                        new FollowPathCommand(follower, stackPath),
                                        new InstantCommand(intake::toggleClamp)
                                ),
                                new FixedSequentialCommandGroup(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.9),
                                        new SensorDetectionCommand(clawSensor)
                                                .setCallback(intake::toggleClamp)
                                )
                        ).andThen(new WaitCommand(400)),
                        new FollowPathCommand(follower, backdropPath)
                                .alongWith(
                                        new WaitCommand(700)
                                                .andThen(new InstantCommand(intake::toggleClamp)),

                                        new WaitUntilCommand(() -> follower.getPose().getY() > 72)
                                                .andThen(
                                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                                        new WaitCommand(300),
                                                        new InstantCommand(() -> {
                                                            intake.adjustLiftPosition(10.0);
                                                            intake.setClampPosition(25);

                                                            outtake.setStopperPositions(DepositSubsystem.Blocker.TWO_PIXELS);
                                                            outtake.setSlidesTicks(200);
                                                            outtake.toggleSpike();
                                                        })
                                                )
                                ),
                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.ONE_PIXEL))
                                .andThen(
                                        new WaitCommand(300),
                                        new InstantCommand(outtake::toggleSpike),
                                        new WaitCommand(200),
                                        new InstantCommand(outtake::toggleSpike),
                                        new WaitCommand(300)
                                ),
                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.FREE))
                                .andThen(
                                        new WaitCommand(300),
                                        new InstantCommand(outtake::toggleSpike),
                                        new InstantCommand(() -> outtake.setSlidesPosition(0))
                                ),

                        // Stack two
                        new ParallelRaceGroup(
                                new FixedSequentialCommandGroup(
                                        new FollowPathCommand(follower, stackPath),
                                        new InstantCommand(intake::toggleClamp)
                                ),
                                new FixedSequentialCommandGroup(
                                        new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.9),
                                        new SensorDetectionCommand(clawSensor)
                                                .setCallback(intake::toggleClamp)
                                )
                        ).andThen(new WaitCommand(400)),
                        new FollowPathCommand(follower, backdropPath)
                                .alongWith(
                                        new WaitCommand(700)
                                                .andThen(new InstantCommand(intake::toggleClamp)),

                                        new WaitUntilCommand(() -> follower.getPose().getY() > 72)
                                                .andThen(
                                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                                        new WaitCommand(300),
                                                        new InstantCommand(() -> {
                                                            intake.adjustLiftPosition(10.0);
                                                            intake.setClampPosition(25);

                                                            outtake.setStopperPositions(DepositSubsystem.Blocker.TWO_PIXELS);
                                                            outtake.setSlidesTicks(300);
                                                            outtake.toggleSpike();
                                                        })
                                                )
                                ),
                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.ONE_PIXEL))
                                .andThen(
                                        new WaitCommand(300),
                                        new InstantCommand(outtake::toggleSpike),
                                        new WaitCommand(200),
                                        new InstantCommand(outtake::toggleSpike),
                                        new WaitCommand(300)
                                ),
                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.FREE))
                                .andThen(
                                        new WaitCommand(300),
                                        new InstantCommand(outtake::toggleSpike),
                                        new InstantCommand(() -> outtake.setSlidesPosition(0))
                                )
                )
        );
    }
}
