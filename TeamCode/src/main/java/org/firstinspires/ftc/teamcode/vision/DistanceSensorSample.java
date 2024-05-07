package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcontroller.internal.DebugOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

@DebugOpMode(group = "Sensors", type = OpModeMeta.Flavor.TELEOP)
public class DistanceSensorSample extends CommandOpMode {

    public void initialize() {
        Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, "distance_wall");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);
        sensor.initialize();

        schedule(new RunCommand(() -> {
            telemetry.addData("Sensor Distance (CM)", sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }));
    }
}
