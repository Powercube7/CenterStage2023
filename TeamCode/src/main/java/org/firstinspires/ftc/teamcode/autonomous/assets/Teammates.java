package org.firstinspires.ftc.teamcode.autonomous.assets;

import android.util.Log;

import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.util.Timing;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import kotlin.Triple;

public enum Teammates {
    SILVER_WOLVES(new Triple<>(9, 10, 9)),
    JUNIPER(new Triple<>(0, 12, 10));
    private final Map<PropLocations, Integer> timerLengths;
    private Timing.Timer autoTimer = new Timing.Timer(0);
    private int currentDuration = 0;


    Teammates(Triple<Integer, Integer, Integer> timerLengths) {
        this.timerLengths = new HashMap<PropLocations, Integer>() {{
            put(PropLocations.LEFT, timerLengths.getFirst());
            put(PropLocations.MIDDLE, timerLengths.getSecond());
            put(PropLocations.RIGHT, timerLengths.getThird());
        }};
    }

    public void setTimerCase(PropLocations propLocation) {
        currentDuration = timerLengths.get(propLocation);
        autoTimer = new Timing.Timer(currentDuration, TimeUnit.SECONDS);
        Log.println(Log.DEBUG, "Teammate", String.format(Locale.ROOT,
                "Timer case has been set to %s. The new duration is %d seconds.",
                propLocation.toString(), currentDuration));
    }

    public void startAutonomous() {
        autoTimer.start();
    }

    public WaitUntilCommand awaitYellow() {
        assert autoTimer.isTimerOn() : "The autonomous timer wasn't started.";
        return new WaitUntilCommand(autoTimer::done);
    }

    public CycleOutput checkCycleTime() {
        if (currentDuration == 0)
            return CycleOutput.FULL_CYCLE;
        if (currentDuration <= 10)
            return CycleOutput.THROW_DOWN;

        return CycleOutput.STOP;
    }

    public enum CycleOutput {
        FULL_CYCLE, THROW_DOWN, STOP
    }
}
