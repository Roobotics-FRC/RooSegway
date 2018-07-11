package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

    private WPI_TalonSRX left1;
    private WPI_TalonSRX left2;
    private WPI_TalonSRX right1;
    private WPI_TalonSRX right2;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT);
        this.left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR);
        this.right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT);
        this.right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR);

        // Make sure that the wheels stay still if they are set to 0
        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);

        // Enable follower mode—motors 1 are master
        this.left2.follow(this.left1);
        this.right2.follow(this.right1);

        // Configure sensors and PID
        this.left1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
                RobotMap.talonTimeoutMs);
        this.left1.setSensorPhase(false);
        this.left1.configNominalOutputForward(0, RobotMap.talonTimeoutMs);
        this.left1.configNominalOutputReverse(0, RobotMap.talonTimeoutMs);
        this.left1.configPeakOutputForward(1, RobotMap.talonTimeoutMs);
        this.left1.configPeakOutputReverse(1, RobotMap.talonTimeoutMs);

        this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
                RobotMap.talonTimeoutMs);
        this.right1.setSensorPhase(false);
        this.right1.configNominalOutputForward(0, RobotMap.talonTimeoutMs);
        this.right1.configNominalOutputReverse(0, RobotMap.talonTimeoutMs);
        this.right1.configPeakOutputForward(1, RobotMap.talonTimeoutMs);
        this.right1.configPeakOutputReverse(1, RobotMap.talonTimeoutMs);

        this.left1.config_kF(0, RobotMap.kF, RobotMap.talonTimeoutMs);
        this.left1.config_kP(0, RobotMap.kP, RobotMap.talonTimeoutMs);
        this.left1.config_kI(0, RobotMap.kI, RobotMap.talonTimeoutMs);
        this.left1.config_kD(0, RobotMap.kD, RobotMap.talonTimeoutMs);

        this.right1.config_kF(0, RobotMap.kF, RobotMap.talonTimeoutMs);
        this.right1.config_kP(0, RobotMap.kP, RobotMap.talonTimeoutMs);
        this.right1.config_kI(0, RobotMap.kI, RobotMap.talonTimeoutMs);
        this.right1.config_kD(0, RobotMap.kD, RobotMap.talonTimeoutMs);
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
     * Gets the current power to the right motor from -1 to 1.
     * @return current power to right motor.
     */
    public double getRight() {
        return right1.get();
    }

    /**
     * Gets the current power to the left motor from -1 to 1.
     * @return current power to left motor.
     */
    public double getLeft() {
        return left1.get();
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
