package org.firstinspires.ftc.teamcode.roadrunner.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.DebugOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.commands.subsystems.CollectorSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;

@Config
@DebugOpMode(group = "debugging", type = OpModeMeta.Flavor.TELEOP)
public class MotorDirectionDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        CollectorSubsystem intake = new CollectorSubsystem(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry.addLine("Press play to begin the debugging OpMode");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clearAll();

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("Xbox/PS4 Button - Motor");
            telemetry.addLine("X / ▢ - Front Left");
            telemetry.addLine("Y / Δ - Front Right");
            telemetry.addLine("B / O - Rear Right");
            telemetry.addLine("A / X - Rear Left");
            telemetry.addLine();

            if(gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0, 0);
                telemetry.addLine("Running Motor: Front Left");
            } else if(gamepad1.y) {
                drive.setMotorPowers(0, 0, 0, MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            } else if(gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER, 0);
                telemetry.addLine("Running Motor: Rear Right");
            } else if(gamepad1.a) {
                drive.setMotorPowers(0, MOTOR_POWER, 0, 0);
                telemetry.addLine("Running Motor: Rear Left");
            } else {
                drive.setMotorPowers(0, 0, 0, 0);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.update();
        }
    }
}
