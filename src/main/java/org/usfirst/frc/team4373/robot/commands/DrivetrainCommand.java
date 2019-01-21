package org.usfirst.frc.team4373.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4373.robot.OI;
import org.usfirst.frc.team4373.robot.input.hid.RooJoystick;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class DrivetrainCommand extends Command {

    private Drivetrain drivetrain;
    private RooJoystick joystick;

    public DrivetrainCommand() {
        requires(this.drivetrain = Drivetrain.getInstance());
        this.joystick = OI.getOI().getDriveJoystick();
    }

    @Override
    public void execute() {
        double y = OI.getOI().getDriveJoystick().rooGetY();
        double z = OI.getOI().getDriveJoystick().rooGetZ();
        this.drivetrain.setPercentOutput(Drivetrain.MotorID.RIGHT_1, y + z);
        this.drivetrain.setPercentOutput(Drivetrain.MotorID.LEFT_1, y - z);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        this.end();
    }

}
