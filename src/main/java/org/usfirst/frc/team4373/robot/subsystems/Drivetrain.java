package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Servo;
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

    public enum Gear {
        LOW, HIGH
    }

    private WPI_TalonSRX left1;
    private WPI_TalonSRX left2;
    private WPI_TalonSRX right1;
    private WPI_TalonSRX right2;
    private Servo rightServo;
    private Servo leftServo;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.left1 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_FRONT);
        this.left2 = new WPI_TalonSRX(RobotMap.LEFT_DRIVE_MOTOR_REAR);
        this.right1 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_FRONT);
        this.right2 = new WPI_TalonSRX(RobotMap.RIGHT_DRIVE_MOTOR_REAR);

        this.left1.setInverted(true);
        this.left2.setInverted(true);

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
                RobotMap.TALON_TIMEOUT_MS);
        this.left1.setSensorPhase(false);
        this.left1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);

        this.right1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,
                RobotMap.TALON_TIMEOUT_MS);
        this.right1.setSensorPhase(false);

        this.right1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS);
        this.right2.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS);
        this.left2.configPeakOutputForward(1, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS);
        this.right2.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS);
        this.left2.configPeakOutputReverse(-1, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.right2.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.left2.configNominalOutputForward(0, RobotMap.TALON_TIMEOUT_MS);
        this.right1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);
        this.right2.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);
        this.left1.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);
        this.left2.configNominalOutputReverse(0, RobotMap.TALON_TIMEOUT_MS);

        this.rightServo = new Servo(RobotMap.RIGHT_SERVO_PORT);
        this.leftServo = new Servo(RobotMap.LEFT_SERVO_PORT);
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
     * Shifts both gearboxes into the specified gear.
     * @param gear the enum Gear state into which to switch.
     */
    public void shift(Gear gear) {
        switch (gear) {
            case LOW:
                this.leftServo.set(0);
                this.rightServo.set(0);
                break;
            case HIGH:
                this.leftServo.set(1);
                this.rightServo.set(1);
                break;
            default:
                break;
        }
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
