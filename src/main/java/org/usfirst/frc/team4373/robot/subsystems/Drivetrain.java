package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.commands.teleop.JoystickControl;

import static org.usfirst.frc.team4373.robot.input.hid.Motors.safetyCheckSpeed;

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

        /*
        Make sure that the wheels stay still if they are set to 0.
         */
        this.left1.setNeutralMode(NeutralMode.Brake);
        this.left2.setNeutralMode(NeutralMode.Brake);
        this.right1.setNeutralMode(NeutralMode.Brake);
        this.right2.setNeutralMode(NeutralMode.Brake);

        this.left2.follow(this.left1);
        this.right2.follow(this.right1);
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

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new JoystickControl());
    }
}
