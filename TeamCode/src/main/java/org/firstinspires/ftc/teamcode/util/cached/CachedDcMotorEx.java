package org.firstinspires.ftc.teamcode.util.cached;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CachedDcMotorEx extends CachedDcMotor implements DcMotorEx {
    public final DcMotorEx motorEx;

    /**
     * Default constructor for the cached motorEx, sets the threshold to 0.02
     *
     * @param motorEx the motor to encapsulate in the caching control
     */
    public CachedDcMotorEx(DcMotorEx motorEx) {
        super(motorEx);
        this.motorEx = motorEx;
    }

    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param motorEx         the motor to encapsulate in the caching control
     * @param changeThreshold the threshold at which the cache should write new values to the motor
     */
    public CachedDcMotorEx(DcMotorEx motorEx, double changeThreshold) {
        super(motorEx, changeThreshold);
        this.motorEx = motorEx;
    }

    /**
     * Individually energizes this particular motor
     *
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorEnable() {
        motorEx.setMotorEnable();
    }

    /**
     * Individually de-energizes this particular motor
     *
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    @Override
    public void setMotorDisable() {
        motorEx.setMotorDisable();
    }

    /**
     * Returns whether this motor is energized
     *
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    @Override
    public boolean isMotorEnabled() {
        return motorEx.isMotorEnabled();
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired angular rate, in units per second
     * @param unit        the units in which angularRate is expressed
     * @see #getVelocity(AngleUnit)
     */
    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        motorEx.setVelocity(angularRate, unit);
    }

    /**
     * Returns the current velocity of the motor, in ticks per second
     *
     * @return the current velocity of the motor
     */
    @Override
    public double getVelocity() {
        return motorEx.getVelocity();
    }

    /**
     * Sets the velocity of the motor
     *
     * @param angularRate the desired ticks per second
     */
    @Override
    public void setVelocity(double angularRate) {
        motorEx.setVelocity(angularRate);
    }

    /**
     * Returns the current velocity of the motor, in angular units per second
     *
     * @param unit the units in which the angular rate is desired
     * @return the current velocity of the motor
     * @see #setVelocity(double, AngleUnit)
     */
    @Override
    public double getVelocity(AngleUnit unit) {
        return motorEx.getVelocity(unit);
    }

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note that in some controller implementations, setting the PID coefficients for one
     * mode on a motor might affect other modes on that motor, or might affect the PID
     * coefficients used by other motors on the same controller (this is not true on the
     * REV Expansion Hub).
     *
     * @param mode            either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     * @see #getPIDCoefficients(RunMode)
     * @deprecated Use {@link #setPIDFCoefficients(RunMode, PIDFCoefficients)} instead
     */
    @Override
    @Deprecated
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        motorEx.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        motorEx.setPIDFCoefficients(mode, pidfCoefficients);
    }


    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        motorEx.setVelocityPIDFCoefficients(p, i, d, f);
    }


    @Override
    public void setPositionPIDFCoefficients(double p) {
        motorEx.setPositionPIDFCoefficients(p);
    }

    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PID control coefficients used when running in the indicated mode on this motor
     * @deprecated Use {@link #getPIDFCoefficients(RunMode)} instead
     */
    @Override
    @Deprecated
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return motorEx.getPIDCoefficients(mode);
    }

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return motorEx.getPIDFCoefficients(mode);
    }

    /**
     * Returns the current target positioning tolerance of this motor
     *
     * @return the current target positioning tolerance of this motor
     */
    @Override
    public int getTargetPositionTolerance() {
        return motorEx.getTargetPositionTolerance();
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        motorEx.setTargetPositionTolerance(tolerance);
    }

    /**
     * Returns the current consumed by this motor.
     *
     * @param unit current units
     * @return the current consumed by this motor.
     */
    @Override
    public double getCurrent(CurrentUnit unit) {
        return motorEx.getCurrent(unit);
    }

    /**
     * Returns the current alert for this motor.
     *
     * @param unit current units
     * @return the current alert for this motor
     */
    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return motorEx.getCurrentAlert(unit);
    }

    /**
     * Sets the current alert for this motor
     *
     * @param current current alert
     * @param unit    current units
     */
    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {
        motorEx.setCurrentAlert(current, unit);
    }

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     *
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    @Override
    public boolean isOverCurrent() {
        return motorEx.isOverCurrent();
    }
}