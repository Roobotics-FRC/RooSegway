package org.usfirst.frc.team4373.robot.commands.profiles;

public interface MotionProfile {
    /**
     * Gets the number of points stored.
     * @return number of points in the array.
     */
    int getNumPoints();

    /**
     * The points in the profile.
     * @return the points.
     */
    double[][] getPoints(); // [[Position, Velocity, Duration]]
}
