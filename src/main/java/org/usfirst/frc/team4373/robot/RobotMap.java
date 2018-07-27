package org.usfirst.frc.team4373.robot;

/**
 * Holds various mappings and constants.
 * @author aaplmath
 */
public class RobotMap {
    // Sensor and OI ports
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;
    public static final int GYRO_CHANNEL = 0;

    // Motor ports
    public static final int LEFT_DRIVE_MOTOR_FRONT = 3; // Dummy
    public static final int LEFT_DRIVE_MOTOR_REAR = 5; // Dummy
    public static final int RIGHT_DRIVE_MOTOR_FRONT = 4; // Dummy
    public static final int RIGHT_DRIVE_MOTOR_REAR = 9; // Dummy

    // PID-related costants
    public static final double kF = 0.982; // At 100% output, 503 native units = 512 sensor units
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int talonTimeoutMs = 1000;
}
