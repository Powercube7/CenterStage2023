package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Matrix;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;

import java.util.concurrent.TimeUnit;

@Config
public class ThreeWheelLocalizer extends Localizer {
    public static double FORWARD_TICKS_TO_INCHES = 5.292746534365396E-4;
    public static double STRAFE_TICKS_TO_INCHES = -5.314807892577837E-4;
    public static double TURN_TICKS_TO_RADIANS = 5.188829415175665E-4;

    public static double LATERAL_DISTANCE = 24.5; // cm
    public static double FORWARD_OFFSET = 12.925; // cm
    public static boolean useIMU = true;
    private final Timing.Timer IMUcooldown = new Timing.Timer(500, TimeUnit.MILLISECONDS);
    private final NanoTimer timer;

    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Matrix prevRotationMatrix;
    private final Encoder leftEncoder, rightEncoder, strafeEncoder;
    private long deltaTimeNano;
    private final Pose leftEncoderPose, rightEncoderPose, strafeEncoderPose;
    private final Telemetry dashboard = FtcDashboard.getInstance().getTelemetry();

    private double totalHeading;
    public IMU imu;

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public ThreeWheelLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new ThreeWheelLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param hardwareMap the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public ThreeWheelLocalizer(HardwareMap hardwareMap, Pose setStartPose) {
        new OdometrySubsystem(hardwareMap).lower();
        IMUcooldown.start();

        imu = hardwareMap.get(IMU.class, "imu_stable");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        leftEncoderPose = new Pose(0, LATERAL_DISTANCE / (2 * 2.54), 0);
        rightEncoderPose = new Pose(0, -LATERAL_DISTANCE / (2 * 2.54), 0);
        strafeEncoderPose = new Pose(FORWARD_OFFSET / 2.54, 0, Math.toRadians(90));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));

        leftEncoder.setDirection(Encoder.FORWARD);
        rightEncoder.setDirection(Encoder.REVERSE);
        strafeEncoder.setDirection(Encoder.REVERSE);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
        totalHeading = 0;

        resetEncoders();
        imu.resetYaw();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return MathFunctions.addPoses(startPose, displacementPose);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the Matrix that contains the previous pose's heading rotation.
     *
     * @param heading the rotation of the Matrix
     */
    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3,3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        displacementPose = MathFunctions.subtractPoses(setPose, startPose);
        imu.resetYaw();
        resetEncoders();
    }

    /**
     * This updates the elapsed time timer that keeps track of time between updates, as well as the
     * change position of the Encoders. Then, the robot's global change in position is calculated
     * using the pose exponential method.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        if (useIMU) {
            dashboard.addData("IMU updates in", Math.max(IMUcooldown.remainingTime(), 0) + " ms");
            dashboard.update();
        }

        updateEncoders();
        Matrix robotDeltas = getRobotDeltas();
        Matrix globalDeltas;
        setPrevRotationMatrix(getPose().getHeading());

        Matrix transformation = new Matrix(3,3);
        if (Math.abs(robotDeltas.get(2, 0)) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(0, 1, -robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 0, robotDeltas.get(2, 0) / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(robotDeltas.get(2, 0), 2) / 6.0));
            transformation.set(2, 2, 1.0);
        } else {
            transformation.set(0, 0, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(0, 1, (Math.cos(robotDeltas.get(2, 0)) - 1.0) / robotDeltas.get(2, 0));
            transformation.set(1, 0, (1.0 - Math.cos(robotDeltas.get(2, 0))) / robotDeltas.get(2, 0));
            transformation.set(1, 1, Math.sin(robotDeltas.get(2, 0)) / robotDeltas.get(2, 0));
            transformation.set(2, 2, 1.0);
        }

        globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), robotDeltas);

        displacementPose.add(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        currentVelocity = new Pose(globalDeltas.get(0, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(1, 0) / (deltaTimeNano * Math.pow(10.0, 9)), globalDeltas.get(2, 0) / (deltaTimeNano * Math.pow(10.0, 9)));

        totalHeading += globalDeltas.get(2, 0);

        if (useIMU && IMUcooldown.done()) {
            double IMUHeading = MathFunctions.normalizeAngle(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            displacementPose.setHeading(IMUHeading);
            IMUcooldown.start();
        }
    }

    /**
     * This updates the Encoders.
     */
    public void updateEncoders() {
        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();
    }

    /**
     * This resets the Encoders.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
    }

    /**
     * This calculates the change in position from the perspective of the robot using information
     * from the Encoders.
     *
     * @return returns a Matrix containing the robot relative movement.
     */
    public Matrix getRobotDeltas() {
        Matrix returnMatrix = new Matrix(3,1);
        // x/forward movement
        returnMatrix.set(0,0, FORWARD_TICKS_TO_INCHES * ((rightEncoder.getDeltaPosition() * leftEncoderPose.getY() - leftEncoder.getDeltaPosition() * rightEncoderPose.getY()) / (leftEncoderPose.getY() - rightEncoderPose.getY())));
        //y/strafe movement
        returnMatrix.set(1,0, STRAFE_TICKS_TO_INCHES * (strafeEncoder.getDeltaPosition() - strafeEncoderPose.getX() * ((rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY()))));
        // theta/turning
        returnMatrix.set(2,0, TURN_TICKS_TO_RADIANS * ((rightEncoder.getDeltaPosition() - leftEncoder.getDeltaPosition()) / (leftEncoderPose.getY() - rightEncoderPose.getY())));
        return returnMatrix;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return TURN_TICKS_TO_RADIANS;
    }
}
