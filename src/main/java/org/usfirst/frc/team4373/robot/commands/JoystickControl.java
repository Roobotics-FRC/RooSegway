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
        drivetrain.resetPigeon();
    }

    @Override
    public void execute() {
        double y = OI.getOI().getDriveJoystick().rooGetY();
        double z = OI.getOI().getDriveJoystick().rooGetZ();

        if (!OI.getOI().getDriveJoystick().getRawButton(1)) { // Speed mode

            double targetHeading = SmartDashboard.getNumber("Desired Heading", 0)
                    / 360 * RobotMap.PIGEON_UNITS_PER_ROTATION;

            this.drivetrain.setLeft(ControlMode.Position, targetHeading);

            /* append more signals to print when in speed mode. */
            sb.append("\t\tout:").append(drivetrain.getLeftPercentOutput());
            sb.append("\t\tang:").append(drivetrain.getPigeonYawPitchRoll()[0]);
            sb.append("\t\tsns:").append(drivetrain.left1.getSelectedSensorPosition());
            sb.append("\t\terr:").append(drivetrain.getLeftClosedLoopError());
            sb.append("\t\ttrg:").append(targetHeading);
        } else {
            // FIXME: always returns 0
            // Percent output—fall back on manual
            double curLeft = drivetrain.getLeftPercentOutput();

            double newLeft = y + z;

            double rightDiff = newLeft - curLeft;
            rightDiff = Math.abs(rightDiff) > 0.01 ? Math.copySign(0.01, rightDiff) : rightDiff;

            System.out.println("RIGHT: cur - " + curLeft + "\tdiff - " + rightDiff
                    + "\tnew_sum - " + (curLeft + rightDiff));

            // drivetrain.setLeft(ControlMode.PercentOutput, curLeft + leftDiff);
            // drivetrain.setLeft(ControlMode.PercentOutput, curLeft + rightDiff);
            drivetrain.setLeft(ControlMode.PercentOutput, 1);
        }

        if (++loops >= 10) {
            loops = 0;
            System.out.println(sb.toString());
        }
        sb.setLength(0);

        SmartDashboard.putNumber("Left Velocity", drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("ANG Sensor Units", drivetrain.left1.getSelectedSensorPosition());

        SmartDashboard.putNumber("Left 1 Pos", this.drivetrain.getLeftPosition());
        SmartDashboard.putNumber("Left 1 Vel", this.drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("Left 1 Power", this.drivetrain.getLeftPercentOutput());
        SmartDashboard.putNumber("Left 2 Power", this.drivetrain.getLeft2PercentOutput());
        SmartDashboard.putNumber("PYaw", this.drivetrain.getPigeonYawPitchRoll()[0]);
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

