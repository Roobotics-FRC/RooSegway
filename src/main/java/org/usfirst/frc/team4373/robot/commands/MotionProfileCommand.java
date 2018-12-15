package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team4373.robot.commands.profiles.MotionProfile;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class MotionProfileCommand extends Command {

    Drivetrain drivetrain;

    MotionProfileFeeder feeder;

    boolean initialized = false;

    public MotionProfileCommand(MotionProfile feeder) {
        requires(this.drivetrain = Drivetrain.getInstance());
    }

    @Override
    public void execute() {
        if (!initialized) {
            this.feeder.reset();
            this.feeder.start(0, true);
            initialized = true;
        } else {
            this.drivetrain.setMotionProfileValue(feeder.getSetValue());
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
