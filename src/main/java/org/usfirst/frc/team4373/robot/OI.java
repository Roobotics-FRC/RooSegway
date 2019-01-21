package org.usfirst.frc.team4373.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import org.usfirst.frc.team4373.robot.commands.MotionProfileCommand;
import org.usfirst.frc.team4373.robot.commands.profiles.MotionProfile;
import org.usfirst.frc.team4373.robot.commands.profiles.Test2019Right;
import org.usfirst.frc.team4373.robot.input.filter.FineGrainedPiecewiseFilter;
import org.usfirst.frc.team4373.robot.input.hid.RooJoystick;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

/**
 * OI encapsulates various inputs and outputs.
 *
 * @author aaplmath
 */
public class OI {
    private static OI oi = null;
    private RooJoystick<FineGrainedPiecewiseFilter> driveJoystick;
    private RooJoystick operatorJoystick;
    private Gyro gyro;
    private Button mpButton;

    private OI() {
        this.driveJoystick =
                new RooJoystick<>(RobotMap.DRIVE_JOYSTICK_PORT, new FineGrainedPiecewiseFilter());
        this.operatorJoystick =
                new RooJoystick<>(RobotMap.OPERATOR_JOYSTICK_PORT,
                        new FineGrainedPiecewiseFilter());
        this.gyro = new AnalogGyro(RobotMap.GYRO_CHANNEL);
        this.mpButton = new JoystickButton(this.driveJoystick, 5);
        this.mpButton.whenPressed(new MotionProfileCommand(
                new Drivetrain.MotorID[] {Drivetrain.MotorID.RIGHT_1, Drivetrain.MotorID.LEFT_1},
                new MotionProfile[] {new Test2019Right(), new Test2019Right()}));

    }

    /**
     * The getter for the OI singleton.
     *
     * @return The static OI singleton object.
     */
    public static OI getOI() {
        if (oi == null) {
            synchronized (OI.class) {
                if (oi == null) {
                    oi = new OI();
                }
            }
        }
        return oi;
    }

    /**
     * Gets the drive joystick controlling the robot.
     * @return The drive joystick controlling the robot.
     */
    public RooJoystick getDriveJoystick() {
        return this.driveJoystick;
    }

    /**
     * Gets the operator joystick controlling the robot.
     * @return The operator joystick controlling the robot.
     */
    public RooJoystick getOperatorJoystick() {
        return this.operatorJoystick;
    }

    /**
     * Gets the gyro measuring the robot's direction.
     * @return The gyro measuring the robot's direction.
     */
    public Gyro getGyro() {
        return gyro;
    }

    /**
     * Gets the gyro angle in degrees.
     * @return The gyro angle in degrees, -180 to 180.
     */
    public double getAngleRelative() {
        double angle = getGyro().getAngle();
        double relative = (Math.abs(angle) * 9 / 2) % 180;
        // TODO: Account for 180° boundary case
        relative *= Math.signum(angle);
        return relative;
    }

    /**
     * Gets the gyro angle in native units.
     * @return The gyro angle, where 20 units = 90 degrees.
     */
    public double getAngleAbsolute() {
        return getGyro().getAngle();
    }
}
