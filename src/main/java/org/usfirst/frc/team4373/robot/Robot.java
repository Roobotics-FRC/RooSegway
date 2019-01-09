package org.usfirst.frc.team4373.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

/**
 * This is the main robot class.
 *
 * @author aaplmath
 */
public class Robot extends IterativeRobot {

    @Override
    public void robotInit() {
        Drivetrain.getInstance();
        SmartDashboard.putNumber("% of Full Speed", 0); // rw
        SmartDashboard.putNumber("Target Angle", 0); // rw
    }

    @Override
    public void teleopInit() {
        OI.getOI().getGyro().reset();
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void testInit() {}

    @Override
    public void disabledInit() {}

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void disabledPeriodic() {}
}
