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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocation;
import org.firstinspires.ftc.teamcode.autonomous.assets.StackGenerator;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.SensorDetectionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Disabled
@Config
@Autonomous
public class TestAuto extends CommandOpMode {
    public static AllianceColor allianceColor = AllianceColor.BLUE;
    private final Point stackPoint = new Point(34, 14, Point.CARTESIAN);
    private final Point backdropPoint = new Point(29, 120, Point.CARTESIAN);

    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(allianceColor.getStartingPose());

        StackGenerator stackPaths = new StackGenerator(stackPoint, backdropPoint)
                .setAlliance(allianceColor)
                .setFollower(follower);

        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);
        RevColorSensorV3 clawSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        PropLocation propLocation = PropLocation.TRUSS_SIDE;
        Path purplePath = propLocation.getPurplePath(allianceColor);
        Path yellowPath = propLocation.getYellowPath(allianceColor);

        PathChain firstStackPath = stackPaths.getStackPath(yellowPath.getLastControlPoint(), StackGenerator.Route.WALL_TRUSS);
        PathChain secondStackPath = stackPaths.getStackPath(backdropPoint, StackGenerator.Route.WALL_TRUSS);
        PathChain backdropPath = stackPaths.getBackdropPath(StackGenerator.Route.WALL_TRUSS);

        schedule(
                new RunCommand(follower::update),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

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
                                .alongWith(new InstantCommand(() -> outtake.setSpikePosition(.875))),

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

                        // First stack
                        new ParallelRaceGroup(
                                new FollowPathCommand(follower, firstStackPath),
                                new FixedSequentialCommandGroup(
                                        new WaitUntilCommand(() -> follower.getPose().getY() < 24),
                                        new SensorDetectionCommand(clawSensor)
                                                .withTimeout(3000)
                                )
                        ).andThen(
                                new InstantCommand(intake::toggleClamp),
                                new WaitCommand(400)
                        ),
                        new FollowPathCommand(follower, backdropPath)
                                .alongWith(
                                        new WaitCommand(700)
                                                .andThen(new InstantCommand(intake::toggleClamp)),

                                        new WaitUntilCommand(() -> follower.getPose().getY() > 72)
                                                .andThen(
                                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
                                                        new WaitCommand(300),
                                                        new InstantCommand(() -> {
                                                            intake.adjustLiftPosition(10.5);
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
                                new FollowPathCommand(follower, secondStackPath),
                                new FixedSequentialCommandGroup(
                                        new WaitUntilCommand(() -> follower.getPose().getY() < 24),
                                        new SensorDetectionCommand(clawSensor)
                                                .withTimeout(3000)
                                )
                        ).andThen(
                                new InstantCommand(intake::toggleClamp),
                                new WaitCommand(400)
                        ),
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
                                                            outtake.setSlidesTicks(500);
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
                                        new WaitCommand(200)
                                ),
                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.FREE))
                                .andThen(
                                        new WaitCommand(300),
                                        new InstantCommand(outtake::toggleSpike),
                                        new InstantCommand(() -> outtake.setSlidesPosition(0))
                                )

//                        // Third stack
//                        new ParallelRaceGroup(
//                                new FixedSequentialCommandGroup(
//                                        new FollowPathCommand(follower, secondStackPath),
//                                        new InstantCommand(intake::toggleClamp)
//                                ),
//                                new FixedSequentialCommandGroup(
//                                        new WaitUntilCommand(() -> follower.getCurrentTValue() > 0.9),
//                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.LOWERED)),
//                                        new SensorDetectionCommand(clawSensor)
//                                                .setCallback(intake::toggleClamp)
//                                                .withTimeout(3000)
//                                )
//                        ).andThen(new WaitCommand(400)),
//                        new FollowPathCommand(follower, backdropPath)
//                                .alongWith(
//                                        new WaitCommand(700)
//                                                .andThen(new InstantCommand(intake::toggleClamp)),
//
//                                        new WaitUntilCommand(() -> follower.getPose().getY() > 72)
//                                                .andThen(
//                                                        new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK)),
//                                                        new WaitCommand(300),
//                                                        new InstantCommand(() -> {
//                                                            intake.adjustLiftPosition(10.0);
//                                                            intake.setClampPosition(25);
//
//                                                            outtake.setStopperPositions(DepositSubsystem.Blocker.TWO_PIXELS);
//                                                            outtake.setSlidesTicks(750);
//                                                            outtake.toggleSpike();
//                                                        })
//                                                )
//                                ),
//                        new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.ONE_PIXEL)).andThen(
//                                new WaitCommand(150),
//                                new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.FREE)),
//                                new WaitCommand(300),
//                                new InstantCommand(() ->{
//                                    outtake.toggleSpike();
//                                    outtake.setSlidesPosition(0);
//                                })
//                        ),
//                        new FollowPathCommand(follower, parking)
//                                .setHoldEnd(false)
                )
        );
    }
}
