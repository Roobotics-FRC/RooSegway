package org.usfirst.frc.team4373.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.OI;
import org.usfirst.frc.team4373.robot.RobotMap;
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
        double z = OI.getOI().getDriveJoystick().rooGetZ();

        SmartDashboard.putNumber("Speed Value",
                this.drivetrain.right1.getSelectedSensorVelocity(RobotMap.SPEED_PID_IDX));
        if (OI.getOI().getDriveJoystick().getRawButton(1)) { // Speed mode
            /*
             * 4096 (Units/Rev) * 5300 (RPM) * 1:24 (gearbox ratio) / 600 (unit of 100ms/min)
             * in each direction
             * velocity setpoint is in units/100ms */
            // 1500 RPM in either direction
            double targetVelocityPer100ms = y * 4096 * 5300 / 24 / 600;

            double targetHeading = y * RobotMap.NATIVE_UNITS_PER_ROTATION;

            this.drivetrain.right1.set(ControlMode.Velocity, targetVelocityPer100ms);

            /* append more signals to print when in speed mode. */
            sb.append("\t\tout:").append(drivetrain.getRightPercentOutput());
            sb.append("\t\tspd:").append(drivetrain.getRightVelocity());
            sb.append("\t\terr:").append(drivetrain.getRightClosedLoopError());
            sb.append("\t\ttrg:").append(targetVelocityPer100ms);
        } else {
            // FIXME: always returns 0
            // Percent output—fall back on manual
            double curRight = drivetrain.getRight();

            double newRight = y + z;

            double rightDiff = newRight - curRight;
            rightDiff = Math.abs(rightDiff) > 0.01 ? Math.copySign(0.01, rightDiff) : rightDiff;

            System.out.println("RIGHT: cur - " + curRight + "\tdiff - " + rightDiff
                    + "\tnew_sum - " + (curRight + rightDiff));

            // drivetrain.setLeft(ControlMode.PercentOutput, curLeft + leftDiff);
            // drivetrain.setRight(ControlMode.PercentOutput, curRight + rightDiff);
            drivetrain.setRight(ControlMode.PercentOutput, 1);
        }

        if (++loops >= 10) {
            loops = 0;
            System.out.println(sb.toString());
        }
        sb.setLength(0);

        SmartDashboard.putNumber("Right Velocity", drivetrain.getRightVelocity());

        SmartDashboard.putNumber("Right 1 SPD Pos", this.drivetrain.right1
                .getSelectedSensorPosition(RobotMap.SPEED_PID_IDX));
        SmartDashboard.putNumber("Right 1 SPD Vel", this.drivetrain.right1
                .getSelectedSensorVelocity(RobotMap.SPEED_PID_IDX));
        SmartDashboard.putNumber("Right 1 Power", this.drivetrain.right1.get());
        SmartDashboard.putNumber("Right 2 Power", this.drivetrain.right2.get());
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

