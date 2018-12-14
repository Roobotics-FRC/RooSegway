package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

/**
 * Feeds setpoints from SmartDashboard to TalonSRX PID loops.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class VelocityHeadingSetpointFeeder extends Command {

    private Drivetrain drivetrain;

    public VelocityHeadingSetpointFeeder() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }


    @Override
    public void initialize() {
        this.drivetrain.zeroMotors();
        this.drivetrain.resetPigeon();
    }

    @Override
    public void execute() {
        /*
         * 4096 (Units/Rev) * 5300 (RPM) * 1:24 (gearbox ratio) / 600 (unit of 100ms/min)
         * in each direction
         * velocity setpoint is in units/100ms */
        // 1500 RPM in either direction
        double targetVelocityNative = SmartDashboard.getNumber("% of Full Speed", 0)
                * 4096 * 5300 / 24 / 600;
        double targetAngleNative = SmartDashboard.getNumber("Target Angle", 0)
                / 360 * RobotMap.RESOLUTION_UNITS_PER_ROTATION;

        this.drivetrain.setSetpoints(targetVelocityNative, targetAngleNative);

        SmartDashboard.putNumber("Yaw", this.drivetrain.getPigeonYaw());
        SmartDashboard.putNumber("Right1/Velocity",
                drivetrain.getSensorVelocity(Drivetrain.MotorID.RIGHT_1,
                        RobotMap.VELOCITY_PID_IDX));
        SmartDashboard.putNumber("Left1/Velocity",
                drivetrain.getSensorVelocity(Drivetrain.MotorID.LEFT_1,
                        RobotMap.VELOCITY_PID_IDX));
        SmartDashboard.putNumber("Right1/Angle",
                drivetrain.getSensorPosition(Drivetrain.MotorID.RIGHT_1,
                        RobotMap.HEADING_PID_IDX));
        SmartDashboard.putNumber("Right1/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.RIGHT_1));
        SmartDashboard.putNumber("Right2/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.RIGHT_2));
        SmartDashboard.putNumber("Left1/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.LEFT_1));
        SmartDashboard.putNumber("Left2/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.LEFT_2));
        SmartDashboard.putNumber("Right1/Vel Error",
                this.drivetrain.getClosedLoopError(Drivetrain.MotorID.RIGHT_1,
                        RobotMap.VELOCITY_PID_IDX));
        SmartDashboard.putNumber("Right1/Head Error",
                this.drivetrain.getClosedLoopError(Drivetrain.MotorID.RIGHT_1,
                        RobotMap.HEADING_PID_IDX));
    }

    @Override
    public void interrupted() {
        this.drivetrain.zeroMotors();
    }

    @Override
    public void end() {
        this.drivetrain.zeroMotors();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}

