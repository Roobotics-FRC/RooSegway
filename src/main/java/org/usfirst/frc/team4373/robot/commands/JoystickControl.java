package org.usfirst.frc.team4373.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
    private StringBuilder sb = new StringBuilder();
    private int loops = 0;

    public JoystickControl() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }


    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double y = OI.getOI().getDriveJoystick().rooGetY();
        double z = OI.getOI().getDriveJoystick().rooGetZ(); // TODO: Account for z in closed-loop mode
        sb.append("\tout:").append(drivetrain.getLeftPercentOutput());
        sb.append("\tspd:").append(drivetrain.getLeftVelocity());

        if (OI.getOI().getDriveJoystick().getRawButton(1)) { /* Speed mode */
            /*
             * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
             * velocity setpoint is in units/100ms */
            double targetVelocityPer100ms = y * 4096 * 500.0 / 600; /* 1500 RPM in either direction */
            drivetrain.setLeft(ControlMode.Velocity, targetVelocityPer100ms);
            drivetrain.setRight(ControlMode.Velocity, targetVelocityPer100ms);
            /* append more signals to print when in speed mode. */
            sb.append("\terr:").append(drivetrain.getLeftClosedLoopError());
            sb.append("\ttrg:").append(targetVelocityPer100ms);
        } else {
            // Percent output—fall back on manual
            double curRight = drivetrain.getRightPercentOutput();
            double curLeft = drivetrain.getLeftPercentOutput();

            double newRight = y + z;
            double newLeft = y - z;

            double rightDiff = newRight - curRight;
            rightDiff = Math.abs(rightDiff) > 0.01 ? Math.copySign(0.01, rightDiff) : rightDiff;
            double leftDiff = newLeft - curLeft;
            leftDiff = Math.abs(leftDiff) > 0.01 ? Math.copySign(0.01, leftDiff) : leftDiff;

            drivetrain.setLeft(ControlMode.PercentOutput, curLeft + leftDiff);
            drivetrain.setRight(ControlMode.PercentOutput, curRight + rightDiff);
        }

        if (++loops >= 10) {
            loops = 0;
            System.out.println(sb.toString());
        }
        sb.setLength(0);
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

