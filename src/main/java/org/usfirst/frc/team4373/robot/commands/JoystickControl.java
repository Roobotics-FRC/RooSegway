package org.usfirst.frc.team4373.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.omg.PortableInterceptor.SYSTEM_EXCEPTION;
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
        sb.append("\tout:").append(drivetrain.left1.getMotorOutputPercent());
        sb.append("\tspd:").append(drivetrain.getLeftVelocity());

        if (OI.getOI().getDriveJoystick().getRawButton(1)) { /* Speed mode */
            /*
             * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
             * velocity setpoint is in units/100ms */
            double targetVelocity_UnitsPer100ms = y * 4096 * 500.0 / 600; /* 1500 RPM in either direction */
            drivetrain.left1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            drivetrain.right1.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            /* append more signals to print when in speed mode. */
            sb.append("\terr:").append(drivetrain.left1.getClosedLoopError(0));
            sb.append("\ttrg:").append(targetVelocity_UnitsPer100ms);
        } else {
            /* Percent output mode */
            drivetrain.left1.set(ControlMode.PercentOutput, y);
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

