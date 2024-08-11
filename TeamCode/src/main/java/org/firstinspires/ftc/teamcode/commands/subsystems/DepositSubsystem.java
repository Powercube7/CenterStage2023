package org.firstinspires.ftc.teamcode.commands.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.util.cached.CachedDcMotor;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

@Config
public class DepositSubsystem extends SubsystemBase {
    private final CachedDcMotor slides;
    public static Double LOW_LEFT = 0.0, LOW_RIGHT = 0.05;
    private final int[] slidesPositions = {0, 400, 700, 1000, 1250};
    private Blocker blockerState = Blocker.FREE;
    private boolean raisingSlides = false;
    private final ServoEx leftLift, rightLift;
    public static Double HIGH_LEFT = .75, HIGH_RIGHT = .8;
    private final ServoEx stopperTop, stopperBottom;
    private BooleanSupplier safeToMove = () -> true;

    private DepositSubsystem(ServoEx leftLift, ServoEx rightLift, ServoEx stopperTop, ServoEx stopperBottom, DcMotor slides) {
        this.leftLift = leftLift;
        this.rightLift = rightLift;

        this.stopperTop = stopperTop;
        this.stopperBottom = stopperBottom;
        this.slides = new CachedDcMotor(slides);

        MotorConfigurationType motorConfigurationType = this.slides.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        this.slides.setMotorType(motorConfigurationType);

        this.stopperBottom.setInverted(true);
        this.rightLift.setInverted(true);

        this.stopperTop.turnToAngle(90);
        this.stopperBottom.turnToAngle(60);
        this.toggleSpike();

        this.slides.setDirection(DcMotorSimple.Direction.FORWARD);

        this.slides.setTargetPosition(0);
        this.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.slides.setPower(1.0);
    }

    public Spike spikeState = Spike.RAISED;

    public DepositSubsystem(final HardwareMap hardwareMap) {
        this(
                new SimpleServo(hardwareMap, "depo_left", 0, 220),
                new SimpleServo(hardwareMap, "depo_right", 0, 220),
                new SimpleServo(hardwareMap, "stopper_top", 0, 300),
                new SimpleServo(hardwareMap, "stopper_bottom", 0, 300),
                hardwareMap.dcMotor.get("gli_sus")
        );
    }

    @Override
    public void periodic() {
        if (raisingSlides) {
            raisingSlides = false;

            if (blockerState == Blocker.FREE)
                setStopperPositions(Blocker.TWO_PIXELS);

            if (spikeState != Spike.RAISED)
                toggleSpike();
        }
    }

    public void setSafeguard(BooleanSupplier safeToMove) {
        this.safeToMove = safeToMove;
    }

    public void setSpikePosition(double position) {
        leftLift.setPosition(MathUtils.clamp(position - 0.05, 0, 1));
        rightLift.setPosition(MathUtils.clamp(position, 0, 1));

        spikeState = (position >= HIGH_RIGHT) ? Spike.RAISED : Spike.LOWERED;
    }

    public void setSlidesTicks(int ticks) {
        if (!safeToMove.getAsBoolean())
            return;

        raisingSlides = slides.getTargetPosition() < 100 && ticks > 100;

        slides.setTargetPosition(ticks);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setPower(1);
    }

    public void setSlidesPosition(int position) {
        position = MathUtils.clamp(position, 0, 4);
        setSlidesTicks(slidesPositions[position]);
    }

    public void raiseSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(slidesPositions, ticks);

        setSlidesPosition(current_pos < 0 ? -(current_pos + 1) : (current_pos + 1));
    }

    public void lowerSlidesPosition() {
        int ticks = slides.getTargetPosition();
        int current_pos = Arrays.binarySearch(slidesPositions, ticks);

        setSlidesPosition(current_pos < 0 ? -(current_pos + 1) - 1 : (current_pos - 1));
    }

    public void adjustSlidesTicks(int ticks) {
        setSlidesTicks(slides.getTargetPosition() + ticks);
    }

    public enum Spike {
        RAISED,
        LOWERED
    }

    public void toggleSpike() {
        if (!safeToMove.getAsBoolean())
            return;

        switch (spikeState) {
            case RAISED:
                leftLift.setPosition(LOW_LEFT);
                rightLift.setPosition(LOW_RIGHT);
                spikeState = Spike.LOWERED;
                break;
            case LOWERED:
                leftLift.setPosition(HIGH_LEFT);
                rightLift.setPosition(HIGH_RIGHT);
                spikeState = Spike.RAISED;
                break;
        }
    }

    public void toggleBlockers() {
        switch (blockerState) {
            case FREE:
                setStopperPositions(Blocker.TWO_PIXELS);
                break;
            case ONE_PIXEL:
                setStopperPositions(Blocker.FREE);
                break;
            case TWO_PIXELS:
                setStopperPositions(Blocker.ONE_PIXEL);
                break;
        }
    }

    public void setStopperPositions(Blocker blockerState) {
        stopperBottom.turnToAngle(blockerState.getBlockerPositions().first);
        stopperTop.turnToAngle(blockerState.getBlockerPositions().second);

        this.blockerState = blockerState;
    }

    public enum Blocker {
        TWO_PIXELS(108, 135), ONE_PIXEL(108, 90), FREE(60, 90);

        private final double bottomPos, topPos;

        Blocker(double bottom, double top) {
            bottomPos = bottom;
            topPos = top;
        }

        public Pair<Double, Double> getBlockerPositions() {
            return new Pair<>(bottomPos, topPos);
        }
    }

    public String getBlockerState() {
        return blockerState.toString();
    }
}
