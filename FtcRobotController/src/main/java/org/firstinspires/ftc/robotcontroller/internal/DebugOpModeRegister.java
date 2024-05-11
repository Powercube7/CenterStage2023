package org.firstinspires.ftc.robotcontroller.internal;

import android.util.Log;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.AnnotatedOpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.HashSet;
import java.util.Objects;

import dev.frozenmilk.sinister.SinisterFilter;
import dev.frozenmilk.sinister.apphooks.OpModeRegistrar;
import dev.frozenmilk.sinister.targeting.SearchTarget;
import dev.frozenmilk.sinister.targeting.TeamCodeSearch;

public class DebugOpModeRegister implements SinisterFilter, OpModeRegistrar {
    private static final DebugOpModeRegister INSTANCE = new DebugOpModeRegister();
    private static final SearchTarget SEARCH_TARGET = new TeamCodeSearch();
    private final HashSet<Class<? extends OpMode>> filteredClasses = new HashSet<>();
    private final boolean DISABLED = false;
    private DebugOpModeRegister() {
        registerInstance();
    }

    @NonNull
    @Override
    public SearchTarget getTargets() {
        return SEARCH_TARGET;
    }

    @Override
    public void registerOpModes(@NonNull AnnotatedOpModeManager opModeManager) {
        filteredClasses.forEach(aClass -> opModeManager.register(getOpModeMetadata(aClass), aClass));
    }

    @Override
    public void filter(@NonNull Class<?> aClass) {
        if (DISABLED)
            return;

        if (!OpMode.class.isAssignableFrom(aClass) || !aClass.isAnnotationPresent(DebugOpMode.class))
            return;

        Log.println(Log.DEBUG, "DebugOpModeRegister", "Found DebugOpMode: " + aClass.getSimpleName());
        filteredClasses.add((Class<? extends OpMode>) aClass);
    }

    private OpModeMeta getOpModeMetadata(Class<? extends OpMode> aClass) {
        DebugOpMode metadata = aClass.getAnnotation(DebugOpMode.class);
        assert metadata != null : "No metadata found for class " + aClass.getSimpleName();
        String className = Objects.equals(metadata.name(), "") ? aClass.getSimpleName() : metadata.name();

        return new OpModeMeta.Builder()
                .setName(className)
                .setFlavor(metadata.type())
                .setSource(metadata.source())
                .setGroup(metadata.group())
                .build();
    }
}
