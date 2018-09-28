package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

    // TODO: Once we've dealt with PID, we can make these private & expose setpoint-setting methods
    public WPI_TalonSRX right1;
    public WPI_TalonSRX right2;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT);
        this.right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR);

        // Make sure that the wheels stay still if they are set to 0
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);

        // Invert all motors
        this.right1.setInverted(true);
        this.right2.setInverted(true);

        // Enable follower mode—motors 1 are master
        this.right2.follow(this.right1);

        catchError(this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
                RobotMap.SPEED_PID_IDX, RobotMap.TALON_TIMEOUT_MS));
        this.right1.setSensorPhase(false);

        catchError(this.right1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS));

        // Configure speed PID
        catchError(this.right1.config_kF(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kF, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kP(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kP, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kI(RobotMap.SPEED_PID_IDX,
                RobotMap.VELOCITY_PID.kI, RobotMap.TALON_TIMEOUT_MS));
        catchError(this.right1.config_kD(RobotMap.SPEED_PID_IDX,
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
     * Gets the percentage of current right motor output.
     * @return current right motor output percentage.
     */
    public double getRightPercentOutput() {
        return right1.getMotorOutputPercent();
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

    public double getRightClosedLoopError() {
        return right1.getClosedLoopError(RobotMap.SPEED_PID_IDX);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickControl());
    }
}
