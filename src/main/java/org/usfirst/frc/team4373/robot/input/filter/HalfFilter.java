package org.usfirst.frc.team4373.robot.input.filter;

/**
 * HalfFilter simply divides input by 2.
 *
 * @author Henry Pitcairn
 * @author Rui-Jie Fang
 */
public class HalfFilter extends DoubleTypeFilter {

    @Override
    public Double applyFilter(Double input) {
        return input / 2;
    }
}
