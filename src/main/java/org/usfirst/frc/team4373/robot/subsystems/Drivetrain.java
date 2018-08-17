package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
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

    public WPI_TalonSRX left1;
    private WPI_TalonSRX left2;
    public WPI_TalonSRX right1;
    private WPI_TalonSRX right2;

    private PigeonIMU leftPigeon;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT);
        this.left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR);
        this.right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT);
        this.right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR);
        this.leftPigeon = new PigeonIMU(this.left2);

        // Make sure that the wheels stay still if they are set to 0
        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);

        // Invert all motors
        this.left1.setInverted(true);
        this.left2.setInverted(true);
        this.right1.setInverted(true);
        this.right2.setInverted(true);

        // Enable follower mode—motors 1 are master
        this.left2.follow(this.left1);
        this.right2.follow(this.right1);

        this.left1.setSensorPhase(false);
        this.right1.setSensorPhase(false);

        // Echo left to remote 0
        this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                RobotMap.SPEED_PID_IDX,
                RobotMap.TALON_TIMEOUT_MS);
        this.right1.configRemoteFeedbackFilter(this.left1.getDeviceID(),
                RemoteSensorSource.TalonSRX_SelectedSensor,
                RobotMap.REMOTE_SENSOR_0,
                RobotMap.TALON_TIMEOUT_MS);
        // Set up difference
        this.right1.configSensorTerm(SensorTerm.Diff1,
                FeedbackDevice.RemoteSensor0,
                RobotMap.TALON_TIMEOUT_MS);
        this.right1.configSensorTerm(SensorTerm.Diff0,
                FeedbackDevice.QuadEncoder,
                RobotMap.TALON_TIMEOUT_MS);
        // Use difference output as sensor
        this.right1.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference,
                RobotMap.SPEED_PID_IDX,
                RobotMap.TALON_TIMEOUT_MS);

        this.right1.configRemoteFeedbackFilter(leftPigeon.getDeviceID(),
                RemoteSensorSource.GadgeteerPigeon_Yaw, RobotMap.REMOTE_SENSOR_0);

        this.right1.configSelectedFeedbackCoefficient(1.0,
                RobotMap.SPEED_PID_IDX, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1,
                RobotMap.HEADING_PID_IDX, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configSelectedFeedbackCoefficient(
                RobotMap.NATIVE_UNITS_PER_ROTATION / RobotMap.PIGEON_UNITS_PER_ROTATION,
                RobotMap.HEADING_PID_IDX, RobotMap.HEADING_PID_IDX);


        this.left1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS);

        this.right1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS);

        // Configure speed PID
        this.right1.config_kF(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS);
        this.right1.config_kP(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS);
        this.right1.config_kI(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS);
        this.right1.config_kD(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kD, RobotMap.TALON_TIMEOUT_MS);

        this.right1.config_kF(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS);
        this.right1.config_kP(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS);
        this.right1.config_kI(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS);
        this.right1.config_kD(RobotMap.HEADING_PID_IDX,
                RobotMap.VELOCITY_PID.kD, RobotMap.TALON_TIMEOUT_MS);

    }

    // -- DEBUG METHOD --

    /**
     * Gets the yaw, pitch, and roll from the Pigeon.
     * @return a three-element array containing in indices 0, 1, and 2 the current
     yaw, pitch, and roll, respectively.
     */
    public double[] getPigeonYawPitchRoll() {
        double[] arr = new double[3];
        this.leftPigeon.getYawPitchRoll(arr);
        return arr;
    }

    /**
     * Sets the left wheels to the specified power.
     * Positive values will make the robot go forward.
     *
     * @param power The power, from -1 to 1, to which to set the motor.
     *              This value is safety checked to make sure it is not out of this range.
     */
    public void setLeft(double power) {
        power = safetyCheckSpeed(power);
        this.left1.set(power);
    }

    /**
     * Sets the left wheels to the specified power in the specified control mode.
     * @param controlMode the mode in which to control the motors.
     * @param power the power (mode-specific) to supply to the motors.
     */
    public void setLeft(ControlMode controlMode, double power) {
        if (controlMode == ControlMode.PercentOutput) {
            power = safetyCheckSpeed(power);
        }
        this.left1.set(controlMode, power);
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
        this.setLeft(power);
        this.setRight(power);
    }

    /**
     * Gets the left motor power.
     * @return left motor power on scale from -1 to 1.
     */
    public double getLeft() {
        return left1.get();
    }

    /**
     * Gets the right motor power.
     * @return right motor power on scale from -1 to 1.
     */
    public double getRight() {
        return right1.get();
    }

    /**
     * Gets the percentage of current left motor output.
     * @return current left motor output percentage.
     */
    public double getLeftPercentOutput() {
        return left1.getMotorOutputPercent();
    }

    /**
     * Gets the percentage of current right motor output.
     * @return current right motor output percentage.
     */
    public double getRightPercentOutput() {
        return right1.getMotorOutputPercent();
    }

    /**
     * Gets the closed-loop error amount from the left motor.
     * @return error amount from left motors.
     */
    public double getLeftClosedLoopError() {
        return left1.getClosedLoopError(0);
    }

    /**
     * Gets the position of the left wheels in units.
     * @return The position of the left wheels, in 'units'.
     */
    public int getLeftPosition() {
        return left1.getSelectedSensorPosition(0);
    }

    /**
     * Gets the velocity of the left wheels in units/0.1s.
     * @return The velocity of the left wheels, in 'units'/0.1s.
     */
    public int getLeftVelocity() {
        return left1.getSelectedSensorVelocity(0);
    }

    /**
     * Gets the position of the right wheels in units.
     * @return The position of the right wheels, in 'units'.
     */
    public int getRightPosition() {
        return right1.getSelectedSensorPosition(0);
    }

    /**
     * Gets the velocity of the right wheels in units/0.1s.
     * @return The velocity of the right wheels, in 'units'/0.1s.
     */
    public int getRightVelocity() {
        return right1.getSelectedSensorVelocity(0);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickControl());
    }
}
