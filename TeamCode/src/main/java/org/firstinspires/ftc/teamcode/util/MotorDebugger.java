package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.cached.CachedDcMotorEx;

@Config
@TeleOp
public class MotorDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 0.7;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = new CachedDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
        leftRear = new CachedDcMotorEx(hardwareMap.get(DcMotorEx.class, "leftBack"));
        rightRear = new CachedDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightBack"));
        rightFront = new CachedDcMotorEx(hardwareMap.get(DcMotorEx.class, "rightFront"));

        waitForStart();
        if (isStopRequested())
            return;

        while (opModeIsActive()) {
            telemetry.addLine("Press each button to turn on its respective motor");
            telemetry.addLine();
            telemetry.addLine("Xbox/PS4 Button - Motor");
            telemetry.addLine("X / ▢ - Front Left");
            telemetry.addLine("Y / Δ - Front Right");
            telemetry.addLine("B / O - Rear Right");
            telemetry.addLine("A / X - Rear Left");
            telemetry.addLine();

            if(gamepad1.x) {
                leftFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Left");
            }
            else leftFront.setPower(0);

            if(gamepad1.y) {
                rightFront.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Front Right");
            }
            else rightFront.setPower(0);

            if(gamepad1.b) {
                rightRear.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Right");
            }
            else rightRear.setPower(0);

            if(gamepad1.a) {
                leftRear.setPower(MOTOR_POWER);
                telemetry.addLine("Running Motor: Rear Left");
            }
            else leftRear.setPower(0);

            telemetry.update();
        }
    }
}
