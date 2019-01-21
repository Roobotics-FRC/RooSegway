package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.commands.profiles.MotionProfile;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class MotionProfileCommand extends Command {

    private Drivetrain drivetrain;

    private Drivetrain.MotorID[] motorIDs;
    private MotionProfileFeeder[] feeders;

    private boolean initialized = false;
    private boolean illegalInitialization;

    /**
     * Instantiates a new MotionProfileCommand.
     * @param motorIDs the array of IDs of the motors to use.
     * @param profs the profiles to use with the motors at the corresponding indices.
     */
    public MotionProfileCommand(Drivetrain.MotorID[] motorIDs, MotionProfile[] profs) {
        illegalInitialization = motorIDs.length != profs.length;
        requires(this.drivetrain = Drivetrain.getInstance());
        this.motorIDs = motorIDs;
        this.feeders = new MotionProfileFeeder[motorIDs.length];
        for (int i = 0; i < motorIDs.length; ++i) {
            this.feeders[i] = new MotionProfileFeeder(
                    this.drivetrain.getTalon(this.motorIDs[i]), profs[i]);
        }
    }

    @Override
    public void execute() {
        for (int i = 0; i < feeders.length; ++i) {
            if (!initialized) {
                this.feeders[i].reset();
                this.feeders[i].start();
                initialized = true;
            } else {
                this.drivetrain.setMotionProfileValue(motorIDs[i],
                        feeders[i].getSetValue());
            }
            feeders[i].control();
        }
        SmartDashboard.putNumber("Left1/Power",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.LEFT_1));
        SmartDashboard.putNumber("Left2/Power",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.LEFT_2));
        SmartDashboard.putNumber("Right1/Power",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.RIGHT_1));
        SmartDashboard.putNumber("Right2/Power",
                this.drivetrain.getOutputPercent(Drivetrain.MotorID.RIGHT_2));
    }

    @Override
    protected boolean isFinished() {
        boolean finished = true;
        for (MotionProfileFeeder feeder : feeders) {
            finished = finished && feeder.isComplete();
        }
        return finished || illegalInitialization;
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
