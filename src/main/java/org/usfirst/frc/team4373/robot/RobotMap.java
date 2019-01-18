package org.usfirst.frc.team4373.robot;

/**
 * Holds various mappings and constants.
 * @author aaplmath
 */
public class RobotMap {

    public static class PID {
        public double kP;
        public double kI;
        public double kD;
        public double kF;

        PID(double kF, double kP, double kI, double kD) {
            this.kF = kF;
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    // Sensor and OI ports
    public static final int DRIVE_JOYSTICK_PORT = 0;
    public static final int OPERATOR_JOYSTICK_PORT = 1;
    public static final int GYRO_CHANNEL = 0;
    public static final int REMOTE_SENSOR_0 = 0;
    public static final int REMOTE_SENSOR_1 = 1;

    // Motor ports
    public static final int LEFT_DRIVE_MOTOR_FRONT = 3;
    public static final int LEFT_DRIVE_MOTOR_REAR = 5;
    public static final int RIGHT_DRIVE_MOTOR_FRONT = 4;
    public static final int RIGHT_DRIVE_MOTOR_REAR = 9;

    // PID-related contants
    public static final PID POSITION_PID = new PID(0, 1, 0, 0);
    public static final int POSITION_PID_IDX = 0;
    public static final int TALON_TIMEOUT_MS = 1000;
    public static final boolean RIGHT_SENSOR_PHASE = false;
    public static final boolean RIGHT_INVERT_MOTOR = true;
    // wheels are 15 inches in diameter, 4096 units = 1 revolution, velocity is in units/0.1sec
    public static final double ENCODER_UNITS_TO_INCHES = 15 * Math.PI / 4096;

    // Turning
    // If these are wrong, blame these people: https://github.com/CrossTheRoadElec/
    // Phoenix-Examples-Languages/blob/a27402ea24c4791c4a38915e07a8155232ab566d/Java/
    // RemoteClosedLoop/src/org/usfirst/frc/team4130/robot/Constants.java
    public static final double PIGEON_UNITS_PER_ROTATION = 8192;
    public static final double RESOLUTION_UNITS_PER_ROTATION = 3600;

}
