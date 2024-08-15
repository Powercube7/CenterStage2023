package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousCommandOpMode;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocation;
import org.firstinspires.ftc.teamcode.autonomous.assets.StackGenerator;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.SensorDetectionCommand;
import org.firstinspires.ftc.teamcode.commands.TensorflowDetectionCommand;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

import java.util.concurrent.Callable;

@Autonomous
public class RedAuto extends AutonomousCommandOpMode {
    private final AllianceColor allianceColor = AllianceColor.RED;
    private final Point stackPoint = new Point(110, 14, Point.CARTESIAN);
    private final Point backdropPoint = new Point(115, 120, Point.CARTESIAN);

    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(allianceColor.getStartingPose());

        StackGenerator stackPaths = new StackGenerator(stackPoint, backdropPoint)
                .setAlliance(allianceColor)
                .setFollower(follower);

        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        DepositSubsystem outtake = new DepositSubsystem(hardwareMap);
        RevColorSensorV3 clawSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");

        TensorflowDetectionCommand propDetection = new TensorflowDetectionCommand(hardwareMap, allianceColor);
        scheduleOnInit(
                new ParallelCommandGroup(
                        propDetection,
                        new RunCommand(() -> {
                            telemetry.addData("Detected Case", propDetection.getPropLocation());
                            telemetry.update();
                        })
                ).interruptOn(this::isStarted)
        );

        Callable<PropLocation> propLocation = propDetection::getPropLocation;
        Callable<Path> purplePath = () -> propLocation.call().getPurplePath(allianceColor);
        Callable<Path> yellowPath = () -> propLocation.call().getYellowPath(allianceColor);

        Callable<PathChain> firstStack = () -> stackPaths.getStackPath(yellowPath.call().getLastControlPoint(),
                propLocation.call() == PropLocation.MIDDLE ? StackGenerator.Route.MIDDLE_TRUSS : StackGenerator.Route.WALL_TRUSS);
        Callable<PathChain> backdrop = () -> stackPaths.getBackdropPath(propLocation.call() == PropLocation.MIDDLE ? StackGenerator.Route.MIDDLE_TRUSS : StackGenerator.Route.WALL_TRUSS);
        Callable<PathChain> secondStack = () -> stackPaths.getStackPath(backdropPoint, propLocation.call() == PropLocation.MIDDLE ? StackGenerator.Route.MIDDLE_TRUSS : StackGenerator.Route.WALL_TRUSS);

        try {
            scheduleOnRun(
                    new RunCommand(follower::update),
                    new FixedSequentialCommandGroup(
                            // Start purple following
                            new InstantCommand(() -> {
                                intake.setLiftLocation(CollectorSubsystem.LiftState.STACK);
                                intake.adjustLiftPosition(10.0);
                            }),
                            new FollowPathCommand(follower, purplePath.call()),

                            // Place purple
                            new InstantCommand(intake::toggleClamp)
                                    .andThen(
                                            new InstantCommand(() -> outtake.setStopperPositions(DepositSubsystem.Blocker.TWO_PIXELS)), // prepare for yellow
                                            new WaitCommand(200),
                                            new InstantCommand(() -> intake.setLiftLocation(CollectorSubsystem.LiftState.STACK))
                                    ),

                            // Head for yellow
                            new FollowPathCommand(follower, yellowPath.call())
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
                                    new FollowPathCommand(follower, firstStack.call())),
                            new FixedSequentialCommandGroup(
                                    new WaitUntilCommand(() -> follower.getPose().getY() < 24),
                                    new SensorDetectionCommand(clawSensor)
                                            .withTimeout(3000)
                            )
                    ).andThen(
                            new InstantCommand(intake::toggleClamp),
                            new WaitCommand(400)
                    ),
                    new FollowPathCommand(follower, backdrop.call())
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
                            new FollowPathCommand(follower, secondStack.call()),
                            new FixedSequentialCommandGroup(
                                    new WaitUntilCommand(() -> follower.getPose().getY() < 24),
                                    new SensorDetectionCommand(clawSensor)
                                            .withTimeout(3000)
                            )
                    ).andThen(
                            new InstantCommand(intake::toggleClamp),
                            new WaitCommand(400)
                    ),
                    new FollowPathCommand(follower, backdrop.call())
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

            );
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}
