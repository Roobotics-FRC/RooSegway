
package org.usfirst.frc.team4373.robot.input.filter;

/**
 * Generic Filter interface
 * All filters shall implement this interface
 * based on a specific type.
 *
 * @author Rui-Jie Fang
 */

public interface GenericFilter<E> {
    public E applyFilter(E input);
}