package org.usfirst.frc.team4373.robot.commands.profiles;

public abstract class MotionProfile {
    public int numPoints;
    public double[][] points; // [[Position, Velocity, Duration]
}
