package org.usfirst.frc.team4373.robot.input.filter;

/**
 * Created by derros on 11/17/17.
 */
public abstract class DoubleTypeFilter implements GenericFilter<Double> {
    public abstract Double applyFilter(Double val);
}
