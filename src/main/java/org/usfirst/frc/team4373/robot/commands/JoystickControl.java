package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.OI;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

/**
 * Provides joystick-based control of the apparatus with a "ramping" input mechanism that caps
 * acceleration at a specific rate.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class JoystickControl extends Command {

    private Drivetrain drivetrain;

    public JoystickControl() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double y = OI.getOI().getDriveJoystick().rooGetY();
        double z = OI.getOI().getDriveJoystick().rooGetZ();

        double curRight = drivetrain.getRight();
        double curLeft = drivetrain.getLeft();

        double newRight = y + z;
        double newLeft = y - z;

        double rightDiff = newRight - curRight;
        rightDiff = Math.abs(rightDiff) > 0.01 ? Math.copySign(0.01, rightDiff) : rightDiff;
        double leftDiff = newLeft - curLeft;
        leftDiff = Math.abs(leftDiff) > 0.01 ? Math.copySign(0.01, leftDiff) : leftDiff;

        drivetrain.setRight(curRight + rightDiff);
        drivetrain.setLeft(curLeft + leftDiff);

        // Shifting debug
        if (OI.getOI().getDriveJoystick().getRawButton(6)) {
            this.drivetrain.shift(Drivetrain.Gear.HIGH);
        } else if (OI.getOI().getDriveJoystick().getRawButton(4)) {
            this.drivetrain.shift(Drivetrain.Gear.LOW);
        }

        SmartDashboard.putNumber("Right Velocity", this.drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Left Velocity", this.drivetrain.getLeftVelocity());
    }

    @Override
    public void interrupted() {
        this.drivetrain.setBoth(0);
    }

    @Override
    public void end() {
        this.drivetrain.setBoth(0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

