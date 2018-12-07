package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4373.robot.OI;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

/**
 * Reads from the joystick and generates velocity and angle setpoints that are passed to the TalonSRX PID loops.
 *
 * @author Samasaur
 */
public class JoystickControl extends Command {
    private Drivetrain drivetrain;

    public JoystickControl() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }


    @Override
    public void initialize() {
        System.out.println("Starting JoystickControl");
        this.drivetrain.zeroMotors();
        this.drivetrain.resetPigeon();
    }

    @Override
    public void execute() {

        double percentOfFullSpeed = 0;
        double targetAngle = 0;

        percentOfFullSpeed = -OI.getOI().getDriveJoystick().rooGetY();
        targetAngle = -(OI.getOI().getDriveJoystick().rooGetTwist() * 180)
                + OI.getOI().getAngleRelative();

        double targetVelocityNative = percentOfFullSpeed * 4096 * 5300 / 24 / 600;
        double targetAngleNative = targetAngle / 360 * RobotMap.RESOLUTION_UNITS_PER_ROTATION;

        this.drivetrain.setSetpoints(targetVelocityNative, targetAngleNative);
    }

    @Override
    public void interrupted() {
        this.end();
    }

    @Override
    public void end() {
        System.out.println("Ending JoystickControl");
        this.drivetrain.zeroMotors();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
