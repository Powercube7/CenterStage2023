package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class AwaitPixelDetectionCommand extends CommandBase {

    private final RevColorSensorV3 sensor;
    private final Command callback, backup;

    public AwaitPixelDetectionCommand(RevColorSensorV3 sensor, Runnable callback) {
        this(sensor, callback, () -> {
        });
    }

    public AwaitPixelDetectionCommand(RevColorSensorV3 sensor, Runnable callback, Runnable backup) {
        this.sensor = sensor;
        this.callback = new InstantCommand(callback);
        this.backup = new InstantCommand(backup);
    }

    public AwaitPixelDetectionCommand(RevColorSensorV3 sensor, Command callback, Command backup) {
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
        if (!interrupted)
            callback.
        else backup.run();
    }
}
