package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DepositSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.EndgameSubsystem;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Tomoiu + Vulpoiu)")
public class TeleOp extends CommandOpMode {
    private List<LynxModule> hubs;
    private CollectorSubsystem intake = null;
    private DepositSubsystem outtake;
    private final int ADJUST_TICKS = 65;

    /**
     * Code to run during the initialization phase of the OpMode.
     */
    public void initialize() {

        this.reset(); // Reset leftover auto commands
        hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        outtake = new DepositSubsystem(hardwareMap);
        OdometrySubsystem odometry = new OdometrySubsystem(this);
        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);
        EndgameSubsystem endgame = new EndgameSubsystem(hardwareMap);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);
        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        Trigger rightTrigger = new Trigger(() -> gamepad2.right_trigger > .3)
                .and(new Trigger(() -> outtake.spikeState == DepositSubsystem.Spike.RAISED));

        register(chassis, outtake, endgame);

        // Brakes
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.5))
                .whenReleased(() -> chassis.setPowerLimit(1.0));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(() -> chassis.setPowerLimit(0.33))
                .whenReleased(() -> chassis.setPowerLimit(1.0));

        // Endgame specific controls
        driver1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(endgame::toggleElevator);
        driver1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(endgame::launchPlane);

        // Slides commands
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(outtake::raiseSlidesPosition);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(outtake::lowerSlidesPosition);

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(4));
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> outtake.setSlidesPosition(0));

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> outtake.adjustSlidesTicks(-ADJUST_TICKS));
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> outtake.adjustSlidesTicks(ADJUST_TICKS));

        // Intake commands
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.adjustLiftPosition(5.0));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.adjustLiftPosition(-5.0));
        driver2.getGamepadButton(GamepadKeys.Button.X)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.toggleLiftLocation());
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> intake != null))
                .whenActive(() -> intake.toggleClamp());

        // Spike commands
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(outtake::toggleSpike);
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(outtake::toggleBlockers);
        rightTrigger.toggleWhenActive(
                () -> outtake.setSpikePosition(.875),
                () -> outtake.setSpikePosition(DepositSubsystem.HIGH_RIGHT)
        );

        schedule(new RunCommand(() -> {
            telemetry.addData("Blocker State", outtake.getBlockerState());
            telemetry.addLine();

            telemetry.addData("Elevator Position", endgame.getElevatorState());
            telemetry.addData("Elevator Angle", endgame.getElevatorAngle());
            telemetry.addLine();

            telemetry.update();
        }));
    }

    /**
     * Loop that runs for every iteration of the OpMode after start is pressed.
     */
    @Override
    public void run() {
        hubs.forEach(LynxModule::clearBulkCache);
        super.run();

        // Stop here if already initialized
        if (intake != null)
            return;

        // Initialize the intake after the OpMode starts to avoid collision with the outtake
        intake = new CollectorSubsystem(hardwareMap);
        outtake.setSafeguard(() -> intake.location != CollectorSubsystem.LiftState.RAISED);
        register(intake);
    }
}
