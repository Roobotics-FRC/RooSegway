package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
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

        // TODO: All of this logic should be ditched in favor of using PID setpoints
        // JavaDoc comments at the top of this file should be modified accordingly once changed
        double rightDiff = newRight - curRight;
        rightDiff = Math.abs(rightDiff) > 0.1 ? Math.copySign(0.1, rightDiff) : rightDiff;
        double leftDiff = newLeft - curLeft;
        leftDiff = Math.abs(leftDiff) > 0.1 ? Math.copySign(0.1, leftDiff) : leftDiff;

        drivetrain.setRight(curRight + rightDiff);
        drivetrain.setLeft(curLeft + leftDiff);
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

