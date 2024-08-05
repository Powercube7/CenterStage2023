package org.firstinspires.ftc.teamcode.util.cached;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CachedDcMotorSimple extends CachingHardwareDevice implements DcMotorSimple {
    public final DcMotorSimple dcMotorSimple;
    double cachedPower;
    double changeThreshold;

    /**
     * Default constructor for the cached simple motor, sets the threshold to 0.05
     *
     * @param motorSimple the simple motor to encapsulate in the caching control
     */
    public CachedDcMotorSimple(DcMotorSimple motorSimple) {
        this(motorSimple, 0.03);
    }

    /**
     * Allows an initial setting of a custom changeThreshold
     *
     * @param motorSimple     the simple motor to encapsulate in the caching control
     * @param changeThreshold the threshold at which the cache should write new values to the motorSimple
     */
    public CachedDcMotorSimple(DcMotorSimple motorSimple, double changeThreshold) {
        super(motorSimple);
        this.dcMotorSimple = motorSimple;
        this.changeThreshold = changeThreshold;
        this.cachedPower = 0.0;
    }

    /**
     * returns the current changeThreshold value
     *
     * @return the current changeThreshold value, defaults to 0.02
     */
    public double getChangeThreshold() {
        return changeThreshold;
    }

    /**
     * Sets the difference between the previously written value and the new value for position before the caching control will write the new value.
     *
     * @param changeThreshold the new change threshold at which the motor will be written to.
     */
    public void setChangeThreshold(double changeThreshold) {
        this.changeThreshold = changeThreshold;
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     *
     * @return the current logical direction in which this motor is set as operating.
     * @see #setDirection(Direction)
     */
    @Override
    public Direction getDirection() {
        return dcMotorSimple.getDirection();
    }

    /**
     * Sets the logical direction in which this motor operates.
     *
     * @param direction the direction to set for this motor
     * @see #getDirection()
     */
    @Override
    public void setDirection(Direction direction) {
        dcMotorSimple.setDirection(direction);
    }

    /**
     * Checks if the change in power output exceeds the set change threshold, if so, does a hardware write
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @return if a hardware write to update the output to the motor was executed
     * @see #setPower(double)
     * @see com.qualcomm.robotcore.hardware.DcMotor#setPower(double)
     */
    public boolean setPowerResult(double power) {
        // will accept the input if it is targeting 0, or full power in any direction, or if it has changed a sufficient amount
        if (Math.abs(power - this.cachedPower) >= changeThreshold || (power == 0.0 && !(cachedPower == 0.0)) || (power >= 1.0 && !(cachedPower >= 1.0)) || (power <= -1.0 && !(cachedPower <= -1.0))) {
            this.cachedPower = power;
            dcMotorSimple.setPower(power);
            return true;
        }
        return false;
    }

    /**
     * Returns the current configured power level of the motor.
     *
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    @Override
    public double getPower() {
        return dcMotorSimple.getPower();
    }

    /**
     * Checks if the change in power output exceeds the set change threshold, if so, does a hardware write
     *
     * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
     * @see com.qualcomm.robotcore.hardware.DcMotor#setPower(double)
     */
    @Override
    public void setPower(double power) {
        // will accept the input if it is targeting 0, or full power in any direction, or if it has changed a sufficient amount
        if (Math.abs(power - this.cachedPower) >= changeThreshold || (power == 0.0 && !(cachedPower == 0.0)) || (power >= 1.0 && !(cachedPower >= 1.0)) || (power <= -1.0 && !(cachedPower <= -1.0))) {
            this.cachedPower = power;
            dcMotorSimple.setPower(power);
        }
    }
}