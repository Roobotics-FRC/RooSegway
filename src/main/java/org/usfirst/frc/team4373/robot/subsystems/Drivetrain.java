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

    public WPI_TalonSRX left1;
    public WPI_TalonSRX left2;

    private PigeonIMU leftPigeon;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT);
        this.left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR);
        this.leftPigeon = new PigeonIMU(this.left2);

        // Make sure that the wheels stay still if they are set to 0
        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);

        // Invert all motors
        this.left1.setInverted(true);
        this.left2.setInverted(true);

        // Enable follower mode—motors 1 are master
        this.left2.follow(this.left1);

        catchError(this.left1.configRemoteFeedbackFilter(leftPigeon.getDeviceID(),
                RemoteSensorSource.GadgeteerPigeon_Yaw, RobotMap.REMOTE_SENSOR_0));
        catchError(this.left1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0,
                RobotMap.SPEED_PID_IDX, RobotMap.TALON_TIMEOUT_MS));
        this.left1.setSensorPhase(false);

        catchError(this.left1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        // Configure speed PID
        catchError(this.left1.config_kF(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.config_kP(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.config_kI(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.left1.config_kD(RobotMap.SPEED_PID_IDX,
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
        this.leftPigeon.getYawPitchRoll(arr);
        return arr;
    }

    /**
     * Sets the left wheels to the specified power.
     * As the motor is inverted, positive values will make the robot go forward.
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
     * Sets the wheels to the specified power.
     * Positive values will make the robot go forward.
     *
     * @param power The power, from -1 to 1, to which to set the motor.
     *              This value is safety checked to make sure it is not out of this range.
     */
    public void setBoth(double power) {
        this.setLeft(power);
    }

    /**
     * Gets the left motor power.
     * @return left motor power on scale from -1 to 1.
     */
    public double getLeft() {
        return left1.get();
    }

    /**
     * Gets the percentage of current left motor output.
     * @return current left motor output percentage.
     */
    public double getLeftPercentOutput() {
        return left1.getMotorOutputPercent();
    }

    // -- DEBUG METHOD --

    /**
     * Gets percent output of secondary left motor.
     * @return percent output of secondary left motor.
     */
    public double getLeft2PercentOutput() {
        return left2.getMotorOutputPercent();
    }

    /**
     * Gets the position of the left wheels in units.
     * @return The position of the left wheels, in 'units'.
     */
    public int getLeftPosition() {
        return left1.getSelectedSensorPosition(RobotMap.SPEED_PID_IDX);
    }

    /**
     * Gets the velocity of the left wheels in units/0.1s.
     * @return The velocity of the left wheels, in 'units'/0.1s.
     */
    public int getLeftVelocity() {
        return left1.getSelectedSensorVelocity(RobotMap.SPEED_PID_IDX);
    }

    public double getLeftClosedLoopError() {
        return left1.getClosedLoopError(RobotMap.SPEED_PID_IDX);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickControl());
    }
}
