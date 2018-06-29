package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4373.robot.OI;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

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
        rightDiff = Math.abs(rightDiff) > 0.1 ? Math.signum(rightDiff) * 0.1 : rightDiff;
        double leftDiff = newLeft - curLeft;
        leftDiff = Math.abs(leftDiff) > 0.1 ? Math.signum(leftDiff) * 0.1 : leftDiff;

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

