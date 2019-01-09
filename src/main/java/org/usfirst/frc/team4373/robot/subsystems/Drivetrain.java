package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.commands.MotionProfileCommand;
import org.usfirst.frc.team4373.robot.commands.VelocityHeadingSetpointFeeder;
import org.usfirst.frc.team4373.robot.commands.profiles.TestProfile;

/**
 * Programmatic representation of physical drivetrain components. Implements TalonSRX-based PID.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class Drivetrain extends Subsystem {

    public enum MotorID {
        RIGHT_1, RIGHT_2, LEFT_1, LEFT_2
    }

    private WPI_TalonSRX right1;
    private WPI_TalonSRX right2;
    private WPI_TalonSRX left1;
    private WPI_TalonSRX left2;

    private PigeonIMU pigeon;

    private int callIdx = 0; // used for ErrorCode catching

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT);
        this.right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR);
        this.left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT);
        this.left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR);
        this.pigeon = new PigeonIMU(this.left2);

        // Make sure that the wheels stay still if they are set to 0
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);
        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);

        // Invert all motors
        this.right1.setInverted(false);
        this.right2.setInverted(false);
        this.left1.setInverted(false);
        this.left2.setInverted(false);

        // Enable follower mode—motors 1 are master
        this.right2.follow(this.right1);
        this.left2.follow(this.left1);

        // Sensor phases
        this.left1.setSensorPhase(false);
        this.left2.setSensorPhase(true);
        this.right1.setSensorPhase(true);

        this.right1.configMotionAcceleration(2000, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configMotionCruiseVelocity(2000, RobotMap.TALON_TIMEOUT_MS);

        // Set up quad encoder on left -> Remote Sensor 0
        catchError(this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                RobotMap.VELOCITY_PID_IDX, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configRemoteFeedbackFilter(this.left1.getDeviceID(),
                RemoteSensorSource.TalonSRX_SelectedSensor,
                RobotMap.REMOTE_SENSOR_0, RobotMap.TALON_TIMEOUT_MS));

        // Set up pigeon on Remote Sensor 1
        catchError(this.right1.configRemoteFeedbackFilter(this.pigeon.getDeviceID(),
                RemoteSensorSource.GadgeteerPigeon_Yaw, RobotMap.REMOTE_SENSOR_1,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
                RobotMap.HEADING_PID_IDX, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSelectedFeedbackCoefficient(
                RobotMap.RESOLUTION_UNITS_PER_ROTATION / RobotMap.PIGEON_UNITS_PER_ROTATION,
                RobotMap.HEADING_PID_IDX, RobotMap.TALON_TIMEOUT_MS));

        // Config averaging from remote sensor and quad encoder on right
        catchError(this.right1.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSelectedFeedbackCoefficient(0.5, RobotMap.VELOCITY_PID_IDX,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSelectedFeedbackSensor(FeedbackDevice.SensorSum,
                RobotMap.VELOCITY_PID_IDX, RobotMap.TALON_TIMEOUT_MS));

        catchError(this.right1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        catchError(this.left1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        // Configure heading PID gains
        catchError(this.right1.config_kF(RobotMap.HEADING_PID_IDX,
                RobotMap.HEADING_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kP(RobotMap.HEADING_PID_IDX,
                RobotMap.HEADING_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kI(RobotMap.HEADING_PID_IDX,
                RobotMap.HEADING_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kD(RobotMap.HEADING_PID_IDX,
                RobotMap.HEADING_PID.kD, RobotMap.TALON_TIMEOUT_MS));

        // Configure speed PID gains
        catchError(this.right1.config_kF(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kP(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kI(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kD(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kD, RobotMap.TALON_TIMEOUT_MS));

        catchError(this.right1.configAuxPIDPolarity(false, RobotMap.TALON_TIMEOUT_MS));
    }

    /**
     * Sets the setpoints of the TalonSRX PID loop to the specified native velocity and heading.
     * @param velocity the velocity setpoint in native units.
     * @param heading the heading setpoint in native units.
     */
    public void setSetpoints(double velocity, double heading) {
        this.right1.set(ControlMode.Velocity, velocity, DemandType.AuxPID, heading);
        this.left1.follow(this.right1, FollowerType.AuxOutput1);
    }

    /**
     * Sets motion profile control mode on a primary motor using auxiliary output.
     * @param primary the primary motor (must be a "1" motor).
     * @param svmpValue the SetValueMotionProfile value to set.
     */
    public void setMotionProfileValue(MotorID primary, SetValueMotionProfile svmpValue) {
        switch (primary) {
            case RIGHT_1:
                this.right1.set(ControlMode.MotionProfileArc, svmpValue.value);
                this.left1.follow(this.right1, FollowerType.AuxOutput1);
                break;
            case LEFT_1:
                this.right1.set(ControlMode.MotionProfileArc, svmpValue.value);
                this.left1.follow(this.right1, FollowerType.AuxOutput1);
                break;
            default:
                break;
        }
    }

    /**
     * Sets all motor outputs to 0.
     */
    public void zeroMotors() {
        this.right1.set(0);
        this.left1.set(0);
    }

    /**
     * Resets the pigeon so that the current heading is 0°.
     */
    public void resetPigeon() {
        ErrorCode code = pigeon.setYaw(0 * 64); // n.b. setYaw() will divide all inputs by 64
        if (code.value != 0) {
            logJNIError("Pigeon reset", code.value);
        }
    }

    /**
     * Returns the yaw value from the Pigeon IMU.
     * @return the yaw value from the Pigeon; 0 if an error occurs.
     */
    public double getPigeonYaw() {
        double[] arr = new double[3];
        ErrorCode code = pigeon.getYawPitchRoll(arr);
        if (code.value != 0) {
            logJNIError("Fetching Pigeon yaw value", code.value);
        } else {
            return arr[0];
        }
        return 0;
    }

    /**
     * Gets the WPI_TalonSRX object associated with the specified motor.
     * <p><b>Note</b>: This method should only be used if direct access to the motor is needed.
     * It is probably preferable to use on of the predefined getters for
     * specific sensor/output values if possible.</p>
     * @param motor the motor to access.
     * @return the object corresponding to the selected motor.
     */
    public WPI_TalonSRX getTalon(MotorID motor) {
        switch (motor) {
            case RIGHT_1:
                return right1;
            case RIGHT_2:
                return right2;
            case LEFT_1:
                return left1;
            case LEFT_2:
                return left2;
            default:
                return null;
        }
    }

    /**
     * Gets the percent output of the specified motor.
     * @param motor the motor whose output percent to get.
     * @return the output percent of the selected motor; 0 if the motor does not exist.
     */
    public double getOutputPercent(MotorID motor) {
        switch (motor) {
            case RIGHT_1:
                return right1.getMotorOutputPercent();
            case RIGHT_2:
                return right2.getMotorOutputPercent();
            case LEFT_1:
                return left1.getMotorOutputPercent();
            case LEFT_2:
                return left2.getMotorOutputPercent();
            default:
                return 0;
        }
    }

    /**
     * Gets the position of the sensor associated with the selected motor on the selected PID loop.
     * @param motor the motor whose sensor to query.
     * @param pidIdx the PID loop to be used.
     * @return the sensor position obtained; 0 if motor is not recognized.
     */
    public double getSensorPosition(MotorID motor, int pidIdx) {
        switch (motor) {
            case RIGHT_1:
                return right1.getSelectedSensorPosition(pidIdx);
            case RIGHT_2:
                return right2.getSelectedSensorPosition(pidIdx);
            case LEFT_1:
                return left1.getSelectedSensorPosition(pidIdx);
            case LEFT_2:
                return left2.getSelectedSensorPosition(pidIdx);
            default:
                return 0;
        }
    }

    /**
     * Gets the velocity of the sensor associated with the selected motor on the selected PID loop.
     * @param motor the motor whose sensor to query.
     * @param pidIdx the PID loop to be used.
     * @return the sensor velocity obtained; 0 if motor is not recognized.
     */
    public double getSensorVelocity(MotorID motor, int pidIdx) {
        switch (motor) {
            case RIGHT_1:
                return right1.getSelectedSensorVelocity(pidIdx);
            case RIGHT_2:
                return right2.getSelectedSensorVelocity(pidIdx);
            case LEFT_1:
                return left1.getSelectedSensorVelocity(pidIdx);
            case LEFT_2:
                return left2.getSelectedSensorVelocity(pidIdx);
            default:
                return 0;
        }
    }

    /**
     * Gets the closed loop error for the selected motor on the selected PID loop.
     * @param motor the motor whose sensor to query.
     * @param pidIdx the PID loop to be used.
     * @return the error value obtained; 0 if motor is not recognized.
     */
    public double getClosedLoopError(MotorID motor, int pidIdx) {
        switch (motor) {
            case RIGHT_1:
                return right1.getClosedLoopError(pidIdx);
            case RIGHT_2:
                return right2.getClosedLoopError(pidIdx);
            case LEFT_1:
                return left1.getClosedLoopError(pidIdx);
            case LEFT_2:
                return left2.getClosedLoopError(pidIdx);
            default:
                return 0;
        }
    }

    /**
     * Catches errors.
     * @param code the ErrorCode object to catch.
     */
    private void catchError(ErrorCode code) {
        if (code.value != 0) {
            logJNIError("Call " + callIdx, code.value);
        } else {
            System.out.println("Call " + callIdx + " succeeded");
        }
        ++callIdx;
    }

    /**
     * Logs a JNI error to DriverStation.
     * @param name the name of the call that failed.
     * @param code the error code generated.
     */
    private void logJNIError(String name, int code) {
        DriverStation.reportError(name + " failed with error code " + code, true);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new VelocityHeadingSetpointFeeder());
    }
}
