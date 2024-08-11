package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SensorDetectionCommand extends CommandBase {

    private final RevColorSensorV3 sensor;
    private Command callback = new InstantCommand();
    private boolean detected = false, initializedCallback = false;

    /**
     * @param sensor   Color sensor to get the distance from
     */
    public SensorDetectionCommand(RevColorSensorV3 sensor) {
        this.sensor = sensor;
    }

    @Override
    public void execute() {
        if (!detected) // Stop checking the sensor when detected to avoid extra hardware reads
            detected = sensor.getDistance(DistanceUnit.CM) < 3.0;

        if (detected) {
            if (!initializedCallback) {
                initializedCallback = true;
                callback.initialize();
            }

            callback.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return detected && callback.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            callback.end(false);
            Log.println(Log.DEBUG, "SensorCommand", "Callback executed successfully.");
        }

        else if (initializedCallback) {
            callback.end(true);
            Log.println(Log.DEBUG, "Sensor Command", "Callback interrupted.");
        }

        else
            Log.println(Log.DEBUG, "Sensor Command", "No detection occurred.");
    }

    public SensorDetectionCommand setCallback(Runnable runnable) {
        this.callback = new InstantCommand(runnable);
        return this;
    }

    public SensorDetectionCommand setCallback(Command command) {
        this.callback = command;
        return this;
    }
}
