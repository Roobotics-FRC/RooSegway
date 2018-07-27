package org.usfirst.frc.team4373.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
        double z = OI.getOI().getDriveJoystick().rooGetZ(); // TODO: Account for z in c-l mode
        sb.append("\tout:").append(drivetrain.getLeftPercentOutput());
        sb.append("\t\tspd:").append(drivetrain.getLeftVelocity());

        if (OI.getOI().getDriveJoystick().getRawButton(1)) { // Speed mode
            /*
             * 4096 (Units/Rev) * 5300 (RPM) * 1:24 (gearbox ratio) / 600 (unit of 100ms/min) in either direction:
             * velocity setpoint is in units/100ms */
            // 1500 RPM in either direction
            // TODO: Replace 1 with y when done debugging
            double targetVelocityPer100ms = 1 * 4096 * 5300 / 24 / 600;
            drivetrain.setLeft(ControlMode.Velocity, targetVelocityPer100ms);
            drivetrain.setRight(ControlMode.Velocity, targetVelocityPer100ms);
            /* append more signals to print when in speed mode. */
            sb.append("\t\terr:").append(drivetrain.getLeftClosedLoopError());
            sb.append("\t\ttrg:").append(targetVelocityPer100ms);
        } else {
            // FIXME: always returns 0
            // Percent output—fall back on manual
            double curLeft = drivetrain.getLeft();
            double curRight = drivetrain.getRight();

            double newLeft = y - z;
            double newRight = y + z;

            double leftDiff = newLeft - curLeft;
            leftDiff = Math.abs(leftDiff) > 0.01 ? Math.copySign(0.01, leftDiff) : leftDiff;
            double rightDiff = newRight - curRight;
            rightDiff = Math.abs(rightDiff) > 0.01 ? Math.copySign(0.01, rightDiff) : rightDiff;

            System.out.println("LEFT: cur - " + curLeft + "\tdiff - " + leftDiff
                    + "\tnew_sum - " + (curLeft + leftDiff));
            System.out.println("RIGHT: cur - " + curRight + "\tdiff - " + rightDiff
                    + "\tnew_sum - " + (curRight + rightDiff));

            drivetrain.setLeft(ControlMode.PercentOutput, curLeft + leftDiff);
            drivetrain.setRight(ControlMode.PercentOutput, curRight + rightDiff);
        }

        if (++loops >= 10) {
            loops = 0;
            System.out.println(sb.toString());
        }
        sb.setLength(0);

        SmartDashboard.putNumber("Right Velocity", drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Left Velocity", drivetrain.getLeftVelocity());
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

