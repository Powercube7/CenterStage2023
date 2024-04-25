package org.firstinspires.ftc.teamcode.autonomous.assets;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

public enum PropLocations {
    LEFT(-1), MIDDLE(0), RIGHT(1);

    public final int id;

    PropLocations(int id) {
        this.id = id;
    }

    @Nullable
    public static PropLocations fromId(int id) {
        for (PropLocations location : PropLocations.values())
            if (location.id == Math.signum(id))
                return location;

        return null;
    }

    @NonNull
    @Override
    public String toString() {
        String name = this.name();
        return name.charAt(0) + name.substring(1).toLowerCase();
    }
}
