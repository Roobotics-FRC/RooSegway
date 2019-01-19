package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.OI;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

import java.io.*;
import java.util.ArrayDeque;
import java.util.HashMap;

/**
 * Provides joystick-based control of the apparatus with a "ramping" input mechanism that caps
 * acceleration at a specific rate.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class JoystickControl extends Command {

    private Drivetrain drivetrain;

    private HashMap<Long, Integer> logs;
    private boolean logged = false;

    public JoystickControl() {
        this.logs = new HashMap();
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

        // double rightDiff = newRight - curRight;
        // rightDiff = Math.abs(rightDiff) > 0.01 ? Math.copySign(0.01, rightDiff) : rightDiff;
        // double leftDiff = newLeft - curLeft;
        // leftDiff = Math.abs(leftDiff) > 0.01 ? Math.copySign(0.01, leftDiff) : leftDiff;
        //
        // drivetrain.setRight(curRight + rightDiff);
        // drivetrain.setLeft(curLeft + leftDiff);
        this.drivetrain.setBoth(y);
        if (OI.getOI().getDriveJoystick().getRawButton(7)) {
            this.drivetrain.setRight(y + z);
            this.drivetrain.setLeft(y - z);
        }

        // Shifting debug
        if (OI.getOI().getDriveJoystick().getRawButton(6)) {
            this.drivetrain.shift(Drivetrain.Gear.HIGH);
        } else if (OI.getOI().getDriveJoystick().getRawButton(4)) {
            this.drivetrain.shift(Drivetrain.Gear.LOW);
        }

        this.logs.put(System.currentTimeMillis(), this.drivetrain.getRightPosition());

        SmartDashboard.putNumber("Right Velocity", this.drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Left Velocity", this.drivetrain.getLeftVelocity());

        // Write out on press of button 10
        if (OI.getOI().getDriveJoystick().getRawButton(10) && !logged) {
            System.out.println("********************************");
            logged = true;
            StringBuilder sb = new StringBuilder();
            logs.forEach((Long time, Integer log) -> {
                sb.append(time.toString());
                sb.append(",");
                sb.append(log.toString());
                sb.append("\n");
            });
            System.out.println(sb.toString());
            System.out.println("********************************");
        }
    }

    @Override
    public void interrupted() {
        this.end();
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

