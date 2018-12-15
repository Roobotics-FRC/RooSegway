package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4373.robot.commands.profiles.MotionProfile;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class MotionProfileCommand extends Command {

    Drivetrain drivetrain;

    Drivetrain.MotorID motorID;
    MotionProfileFeeder feeder;

    boolean initialized = false;

    public MotionProfileCommand(MotionProfile prof, Drivetrain.MotorID motorID) {
        requires(this.drivetrain = Drivetrain.getInstance());
        this.motorID = motorID;
        this.feeder = new MotionProfileFeeder(this.drivetrain.getTalon(this.motorID), prof);
    }

    @Override
    public void execute() {
        if (!initialized) {
            this.feeder.reset();
            this.feeder.start(0, true);
            initialized = true;
        } else {
            this.drivetrain.setMotionProfileValue(motorID,
                    feeder.getSetValue());
        }
        feeder.control();
    }

    @Override
    protected boolean isFinished() {
        return feeder.isComplete();
    }

    @Override
    protected void end() {
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
