package org.firstinspires.ftc.teamcode.purepursuit;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Arrays;
import java.util.List;

public class PurePursuitDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients();
    public static PIDCoefficients HEADING_PID = new PIDCoefficients();
    public static double LATERAL_MULTIPLIER = 1;
    private final Motor leftFront, leftBack, rightFront, rightBack;
    private final VoltageSensor voltageSensor;
    private final List<Motor> motors;

    public PurePursuitDrive(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, "leftFront", 384.5, 435);
        rightFront = new Motor(hardwareMap, "rightFront", 384.5, 435);
        leftBack = new Motor(hardwareMap, "leftBack", 384.5, 435);
        rightBack = new Motor(hardwareMap, "rightBack", 384.5, 435);

        rightFront.setInverted(true);
        rightBack.setInverted(true);

        motors = Arrays.asList(leftFront, rightFront, leftBack, rightBack);
        hardwareMap.getAll(LynxModule.class).forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        motors.forEach(motor -> motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE));
    }

    public void setWeightedDrivePower(double y, double x, double rx) {
        double voltageCompensation = 12.0 / voltageSensor.getVoltage();
        x *= (LATERAL_MULTIPLIER * voltageCompensation);
        y *= voltageCompensation;
        rx *= voltageCompensation;

        double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(rx)), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.set(frontLeftPower);
        rightFront.set(frontRightPower);
        leftBack.set(backLeftPower);
        rightBack.set(backRightPower);
    }
}
