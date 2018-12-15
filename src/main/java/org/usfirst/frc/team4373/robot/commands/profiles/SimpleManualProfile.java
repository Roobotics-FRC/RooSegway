package org.usfirst.frc.team4373.robot.commands.profiles;

public class SimpleManualProfile implements MotionProfile {

    /**
     * Gets the number of points in the profile.
     * @return the number of points in the profile.
     */
    public int getNumPoints() {
        return 2;
    }

    /**
     * Gets the points in the profile.
     * @return the points in the profile.
     */
    public double[][] getPoints() {
        return new double[][]{
                {10, 120, 5000},
                {5, 60, 5000}
        };
    }
}
