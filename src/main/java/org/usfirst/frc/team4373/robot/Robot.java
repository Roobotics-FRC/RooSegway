package org.usfirst.frc.team4373.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
        OI.getOI().getGyro().calibrate();
        // SmartDashboard
        SmartDashboard.putNumber("Desired Heading", 0);
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
