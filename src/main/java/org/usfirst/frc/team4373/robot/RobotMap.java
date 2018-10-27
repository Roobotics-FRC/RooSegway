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
    public static final int LEFT_DRIVE_MOTOR_FRONT = 3; // Dummy
    public static final int LEFT_DRIVE_MOTOR_REAR = 5; // Dummy
    public static final int RIGHT_DRIVE_MOTOR_FRONT = 4; // Dummy
    public static final int RIGHT_DRIVE_MOTOR_REAR = 9; // Dummy

    // PID-related costants
    public static final PID VELOCITY_PID = new PID(0, 1, 0, 0);
    public static final PID HEADING_PID = new PID(0, 1, 0, 0);
    // At 100% output, 503 native units = 512 sensor units
    public static final int VELOCITY_PID_IDX = 0;
    public static final int HEADING_PID_IDX = 0;
    public static final int TALON_TIMEOUT_MS = 1000;

    // Turning
    // If these are wrong, blame these people: https://github.com/CrossTheRoadElec/
    // Phoenix-Examples-Languages/blob/a27402ea24c4791c4a38915e07a8155232ab566d/Java/
    // RemoteClosedLoop/src/org/usfirst/frc/team4130/robot/Constants.java
    public static final double NATIVE_UNITS_PER_ROTATION = 3600;
    public static final double PIGEON_UNITS_PER_ROTATION = 8192;

}
