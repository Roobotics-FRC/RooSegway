package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.Robot;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

/**
 * Feeds setpoints from SmartDashboard to TalonSRX PID loops.
 *
 * @author aaplmath
 * @author Samasaur
 */
public class SetpointFeeder extends Command {

    private Drivetrain drivetrain;

    public SetpointFeeder() {
        requires(this.drivetrain = Drivetrain.getInstance());
    }


    @Override
    public void initialize() {
        this.drivetrain.zeroMotors();
    }

    @Override
    public void execute() {
        double targetPosNative = SmartDashboard.getNumber("Target Position (in)", 0)
                / RobotMap.ENCODER_UNITS_TO_INCHES;

        this.drivetrain.setPosSetpoint(targetPosNative);

        SmartDashboard.putNumber("Right1/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.RIGHT_1));
        SmartDashboard.putNumber("Right2/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.RIGHT_2));
        SmartDashboard.putNumber("Left/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.LEFT_1));
        SmartDashboard.putNumber("Left2/Perc Output",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.LEFT_2));
        SmartDashboard.putNumber("Right1/ClosedLoopErr",
                this.drivetrain.getClosedLoopError(Drivetrain.MotorID.RIGHT_1,
                        RobotMap.POSITION_PID_IDX));
        SmartDashboard.putNumber("Right1/SensorPos",
                this.drivetrain.getSensorPosition(Drivetrain.MotorID.RIGHT_1,
                        RobotMap.POSITION_PID_IDX));
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

