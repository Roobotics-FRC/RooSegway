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
public class FineGrainedPiecewiseFilter extends DoubleTypeFilter {

    @Override
    public Double applyFilter(Double input) {
        return (Math.abs(input) <= 0.8) ? 0.5 * input : ((3 * input) - (Math.signum(input) * 2));
    }
}
