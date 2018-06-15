package org.usfirst.frc.team4373.robot.input.filter;

/**
 * Piecewise linear function for more granular joystick control.
 * -------------------------------------------------------------
 * Note that this piecewise function is pre-defined for a single case.
 * As such, it should MOST LIKELY NOT be used in anything new.
 *
 * @author (tesla)
 * @author Rui-Jie Fang
 */
public class PiecewiseFilter extends DoubleTypeFilter {

    @Override
    public Double applyFilter(Double input) {
        if (Math.abs(input) <= 0.4) {
            return 0.5 * input;
        } else if (Math.abs(input) > 0.4 && Math.abs(input) <= 0.8) {
            return (0.75 * input) - (Math.signum(input) * 0.1);
        } else {
            return (2.5 * input) - (Math.signum(input) * 1.5);
        }
    }

}
