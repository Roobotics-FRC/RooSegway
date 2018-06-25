package org.usfirst.frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team4373.robot.RobotMap;

import static org.usfirst.frc.team4373.robot.input.hid.Motors.safetyCheckSpeed;

/**
 * A programmatic representation of the robot's drivetrain.
 *
 * @author Samasaur
 * @author Benji123abc
 */
public class Drivetrain extends Subsystem {

    //declare variables
    private WPI_TalonSRX left;
    private WPI_TalonSRX right;

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        this.left = new WPI_TalonSRX(RobotMap.LEFT_MOTOR);
        this.right = new WPI_TalonSRX(RobotMap.RIGHT_MOTOR);

        this.left.setNeutralMode(NeutralMode.Brake);
        this.right.setNeutralMode(NeutralMode.Brake);

        this.right.setInverted(true);
    }

    //setting methods go here
    public void setLeft(double power) {
        power = safetyCheckSpeed(power);
        this.left.set(power);
    }

    public void setRight(double power) {
        power = safetyCheckSpeed(power);
        this.right.set(power);
    }

    public void setBoth(double power) {
        setRight(power);
        setLeft(power);
    }

    //getting methods go here

    @Override
    protected void initDefaultCommand() {
        //setDefaultCommand(new BalancingCommand()); //it's a balancing act
    }
}
