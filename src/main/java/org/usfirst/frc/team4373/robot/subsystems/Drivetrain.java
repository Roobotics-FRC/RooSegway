package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.commands.JoystickControl;

import static org.usfirst.frc.team4373.robot.input.hid.Motors.safetyCheckSpeed;

/**
 * Programmatic representation of physical drivetrain components. Implements TalonSRX-based PID.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class Drivetrain extends Subsystem {

    public WPI_TalonSRX right1;
    private WPI_TalonSRX right2;
    public WPI_TalonSRX left1;
    private WPI_TalonSRX left2;

    private PigeonIMU pigeon;

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
        this.right1.setInverted(true);
        this.right2.setInverted(true);
        this.left1.setInverted(false);
        this.left2.setInverted(false);

        // Enable follower mode—motors 1 are master
        this.right2.follow(this.right1);
        this.left2.follow(this.left1);

        // Set up quad encoder on left -> Remote Sensor 0
        catchError(this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                RobotMap.VELOCITY_PID_IDX, RobotMap.TALON_TIMEOUT_MS));
        this.left1.setSensorPhase(false);
        catchError(this.right1.configRemoteFeedbackFilter(this.left1.getDeviceID(),
                RemoteSensorSource.TalonSRX_SelectedSensor,
                RobotMap.REMOTE_SENSOR_0, RobotMap.TALON_TIMEOUT_MS));

        // Set up pigeon on Remote Sensor 1
        catchError(this.right1.configRemoteFeedbackFilter(this.pigeon.getDeviceID(),
                RemoteSensorSource.GadgeteerPigeon_Yaw, RobotMap.REMOTE_SENSOR_1,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
                RobotMap.HEADING_PID_IDX, RobotMap.TALON_TIMEOUT_MS));

        // Config averaging from remote sensor and quad encoder on right
        catchError(this.right1.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder,
                RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configSelectedFeedbackCoefficient(0.5, RobotMap.VELOCITY_PID_IDX,
                RobotMap.TALON_TIMEOUT_MS));

        catchError(this.right1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        catchError(this.left1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        // Configure speed PID
        catchError(this.right1.config_kF(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kP(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kI(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kD(RobotMap.VELOCITY_PID_IDX,
                RobotMap.VELOCITY_PID.kD, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kF(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kP(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kI(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kD(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kD, RobotMap.TALON_TIMEOUT_MS));

    }

    // -- DEBUG METHOD --

    private int callIdx = 0;

    /**
     * Catches errors.
     * @param code the ErrorCode object to catch.
     */
    private void catchError(ErrorCode code) {
        if (code.value != 0) {
            DriverStation.reportError("Call " + callIdx
                    + " failed with error code " + code.value, true);
        } else {
            System.out.println("Call " + callIdx + " succeeded");
        }
        ++callIdx;
    }

    /**
     * Gets the yaw, pitch, and roll from the Pigeon.
     * @return a three-element array containing in indices 0, 1, and 2 the current
    yaw, pitch, and roll, respectively.
     */
    public double[] getPigeonYawPitchRoll() {
        double[] arr = new double[3];
        this.pigeon.getYawPitchRoll(arr);
        return arr;
    }

    /**
     * Sets the right wheels to the specified power.
     * As the motor is inverted, positive values will make the robot go forward.
     *
     * @param power The power, from -1 to 1, to which to set the motor.
     *              This value is safety checked to make sure it is not out of this range.
     */
    public void setRight(double power) {
        power = safetyCheckSpeed(power);
        this.right1.set(power);
    }

    /**
     * Sets the right wheels to the specified power in the specified control mode.
     * @param controlMode the mode in which to control the motors.
     * @param power the power (mode-specific) to supply to the motors.
     */
    public void setRight(ControlMode controlMode, double power) {
        if (controlMode == ControlMode.PercentOutput) {
            power = safetyCheckSpeed(power);
        }
        this.right1.set(controlMode, power);
    }

    /**
     * Sets the wheels to the specified power.
     * Positive values will make the robot go forward.
     *
     * @param power The power, from -1 to 1, to which to set the motor.
     *              This value is safety checked to make sure it is not out of this range.
     */
    public void setBoth(double power) {
        this.setRight(power);
    }

    /**
     * Gets the right motor power.
     * @return right motor power on scale from -1 to 1.
     */
    public double getRight() {
        return right1.get();
    }

    /**
     * Gets the percentage of primary right motor output.
     * @return primary right motor output percentage.
     */
    public double getRight1PercentOutput() {
        return right1.getMotorOutputPercent();
    }

    /**
     * Gets percent output of secondary right motor.
     * @return percent output of secondary right motor.
     */
    public double getRight2PercentOutput() {
        return right2.getMotorOutputPercent();
    }

    /**
     * Gets the percentage of primary right motor output.
     * @return primary right motor output percentage.
     */
    public double getLeft1PercentOutput() {
        return left1.getMotorOutputPercent();
    }

    /**
     * Gets percent output of secondary left motor.
     * @return percent output of secondary left motor.
     */
    public double getLeft2PercentOutput() {
        return left2.getMotorOutputPercent();
    }

    /**
     * Gets the position of the right wheels in units.
     * @return The position of the right wheels, in 'units'.
     */
    public int getRightPosition() {
        return right1.getSelectedSensorPosition(RobotMap.VELOCITY_PID_IDX);
    }

    /**
     * Gets the velocity of the right wheels in units/0.1s.
     * @return The velocity of the right wheels, in 'units'/0.1s.
     */
    public int getRightVelocity() {
        return right1.getSelectedSensorVelocity(RobotMap.VELOCITY_PID_IDX);
    }

    public double getRightClosedLoopError() {
        return right1.getClosedLoopError(RobotMap.VELOCITY_PID_IDX);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickControl());
    }
}
