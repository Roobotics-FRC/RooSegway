package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.commands.SetpointFeeder;

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

    private int callIdx = 0; // used for ErrorCode catching

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        // right1 and right2 are flipped because the rear has the encoder on it
        this.right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR);
        this.right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT);
        this.left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT);
        this.left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR);

        // Make sure that the wheels stay still if they are set to 0
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);
        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);

        // Invert all motors
        this.right1.setInverted(RobotMap.RIGHT_INVERT_MOTOR);
        this.right2.setInverted(RobotMap.RIGHT_INVERT_MOTOR);
        this.left1.setInverted(false);
        this.left2.setInverted(false);

        // Enable follower mode—motors 1 are master
        this.right2.follow(this.right1);
        this.left1.follow(this.right1);
        this.left2.follow(this.right1);

        // Sensor phases
        this.right1.setSensorPhase(RobotMap.RIGHT_SENSOR_PHASE);

        // Set up quad encoder
        catchError(this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                RobotMap.POSITION_PID_IDX, RobotMap.TALON_TIMEOUT_MS));

        catchError(this.right1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        catchError(this.right1.configAllowableClosedloopError(RobotMap.POSITION_PID_IDX,
                0, RobotMap.TALON_TIMEOUT_MS));

        // Configure speed PID gains
        catchError(this.right1.config_kF(RobotMap.POSITION_PID_IDX,
                RobotMap.POSITION_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kP(RobotMap.POSITION_PID_IDX,
                RobotMap.POSITION_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kI(RobotMap.POSITION_PID_IDX,
                RobotMap.POSITION_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kD(RobotMap.POSITION_PID_IDX,
                RobotMap.POSITION_PID.kD, RobotMap.TALON_TIMEOUT_MS));

        int absolutePosition = this.right1.getSensorCollection().getQuadraturePosition();
        absolutePosition &= 0xFFF;
        if (RobotMap.RIGHT_SENSOR_PHASE) absolutePosition *= -1;
        if (RobotMap.RIGHT_INVERT_MOTOR) absolutePosition *= -1;

        catchError(this.right1.setSelectedSensorPosition(absolutePosition,
                RobotMap.POSITION_PID_IDX, RobotMap.TALON_TIMEOUT_MS));
    }

    /**
     * Sets the setpoints of the TalonSRX PID loop to the specified native velocity and heading.
     * @param position the position setpoint in native units.
     */
    public void setPosSetpoint(double position) {
        this.right1.set(ControlMode.Position, position);
    }

    /**
     * Sets all motor outputs to 0.
     */
    public void zeroMotors() {
        this.right1.set(0);
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
        setDefaultCommand(new SetpointFeeder());
    }
}
