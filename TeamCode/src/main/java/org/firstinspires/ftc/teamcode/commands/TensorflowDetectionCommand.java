package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.assets.AllianceColor;
import org.firstinspires.ftc.teamcode.autonomous.assets.PropLocation;
import org.firstinspires.ftc.teamcode.commands.subsystems.TensorflowSubsystem;

import java.util.Locale;

public class TensorflowDetectionCommand extends CommandBase {

    private final TensorflowSubsystem tensorflow;
    private final AllianceColor color;
    private PropLocation propLocation = PropLocation.TRUSS_SIDE;

    public TensorflowDetectionCommand(HardwareMap hardwareMap, AllianceColor color) {
        this.color = color;
        tensorflow = new TensorflowSubsystem(hardwareMap, "Webcam 1",
                color.name().toLowerCase(Locale.ROOT) + "_prop.tflite", "Team Prop");

        tensorflow.setMinConfidence(0.75);
    }

    @Override
    public void execute() {
        Recognition bestDetection = tensorflow.getBestDetection();
        propLocation = PropLocation.TRUSS_SIDE;

        if (bestDetection != null) {
            double x = (bestDetection.getLeft() + bestDetection.getRight()) / 2;

            if (x < (bestDetection.getImageWidth() / 2.0))
                propLocation = (color == AllianceColor.BLUE ? PropLocation.BACKDROP_SIDE : PropLocation.MIDDLE);
            else
                propLocation = (color == AllianceColor.BLUE ? PropLocation.MIDDLE : PropLocation.BACKDROP_SIDE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        tensorflow.shutdown();
    }

    public PropLocation getPropLocation() {
        return propLocation;
    }
}
