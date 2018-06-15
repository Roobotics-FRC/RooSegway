package org.usfirst.frc.team4373.robot.input.hid;

/**
 * This class holds various constants and methods related to motors.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class Motors {

    private Motors() {}

    // Conversion factors to inches or inches/second
    // wheels are 6 inches in diameter, 4096 units = 1 revolution, velocity is in units/0.1sec
    /**
     * When a double is multiplied by this constant, it is converted from 'units' to inches.
     */
    public static final double POSITION_CONVERSION_FACTOR = 6 * Math.PI / 4096;
    /**
     * When a double is multiplied by this constant, it is converted from 'units'/0.1s to inches/s.
     */
    public static final double VELOCITY_CONVERSION_FACTOR = 10 * 6 * Math.PI / 4096;

    /**
     * Make sure a given power is within the valid -1 to 1 range for motors.
     * @param power The power to be safety checked.
     * @return The power, now within a safe -1 to 1 range.
     */
    public static double safetyCheckSpeed(double power) {
        if (power > 1) {
            return 1;
        } else if (power < -1) {
            return -1;
        }
        return power;
    }
}
