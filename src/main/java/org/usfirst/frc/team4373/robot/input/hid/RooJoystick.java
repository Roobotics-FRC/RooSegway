package org.usfirst.frc.team4373.robot.input.hid;

import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc.team4373.robot.input.filter.DoubleTypeFilter;

/**
 * This class extends the WPILib Joystick class
 * to add deadzone and filter functionality.
 * New features:
 * =============
 * 1) Generify the class based on filter type, expressed as an upper bound GenericFilter
 * 2) Rewrite as Singleton
 * 3) Remove custom filter capabilities
 *
 * @author (Henry Pitcairn)
 * @author Rui-Jie Fang
 */
public class RooJoystick<F extends DoubleTypeFilter> extends Joystick {
    private static final double DEADZONE = 0.09;
    private F filter = null;

    public RooJoystick(int port, F filter) {
        super(port);
        this.filter = filter;
    }

    /**
     * Filter a value using the joystick's filter.
     * @param val The value to filter.
     * @return The value post-filtering.
     */
    private double filter(double val) {
        // We don't know the return type because of type erasure...
        return applyDeadzone(this.filter.applyFilter(val));
    }

    /**
     * Ignores input if it is within the deadzone (if it is negligible).
     * @param input The input value to be checked.
     * @return The input value if it is large enough, or 0 if it was negligible.
     */
    private double applyDeadzone(double input) {
        return Math.abs(input) <= DEADZONE ? 0 : input;
    }

    public double rooGetX() {
        return this.filter(this.getX());
    }

    public double rooGetY() {
        return this.filter(this.getY());
    }

    public double rooGetZ() {
        return this.filter(this.getZ());
    }

    public double rooGetTwist() {
        return this.filter(this.getTwist());
    }

    public double rooGetThrottle() {
        return this.filter(this.getThrottle());
    }

    @Override
    public double getAxis(Joystick.AxisType axis) {
        return this.getAxis(axis.value);
    }


    /**
     * Returns the filtered value of a joystick access.
     *
     * @param axis the axis to read from
     * @return the filtered value of the axis
     */
    public double getAxis(int axis) {
        switch (axis) {
            case 0:
                return this.rooGetX();
            case 1:
                return this.rooGetY();
            case 2:
                return this.rooGetZ();
            case 3:
                return this.rooGetTwist();
            case 4:
                return this.rooGetThrottle();
            default:
                return 0.0;
        }
    }

    /**
     * Gets the angle the joystick is facing relative to neutral.
     * @return the joystick angle
     */
    public double getAngle() {
        double x = this.getAxis(AxisType.kX);
        double y = this.getAxis(AxisType.kY);
        return Math.atan(y / x);
    }
}