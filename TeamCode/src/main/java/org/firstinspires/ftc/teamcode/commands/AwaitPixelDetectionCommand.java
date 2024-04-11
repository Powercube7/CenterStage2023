package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AwaitPixelDetectionCommand extends CommandBase {

    private final RevColorSensorV3 sensor;
    private final Runnable callback, backup;

    /**
     * @param sensor   Color sensor to get the distance from
     * @param callback Action to run if a pixel is detected successfully
     */
    public AwaitPixelDetectionCommand(RevColorSensorV3 sensor, Runnable callback) {
        this(sensor, callback, callback);
    }

    /**
     *
     * @param sensor Color sensor to get the distance from
     * @param callback Action to run if a pixel is detected successfully
     * @param backup Action to run if the command is interrupted before finding a pixel
     */
    public AwaitPixelDetectionCommand(RevColorSensorV3 sensor, Runnable callback, Runnable backup) {
        this.sensor = sensor;
        this.callback = callback;
        this.backup = backup;
    }

    @Override
    public boolean isFinished() {
        return sensor.getDistance(DistanceUnit.CM) < 3.0;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            Log.println(Log.DEBUG, "PixelCommand", "Pixel detected successfully. Running callback...");
            callback.run();
        } else {
            Log.println(Log.DEBUG, "PixelCommand", "No pixels detected. Running backup...");
            backup.run();
        }
    }
}
