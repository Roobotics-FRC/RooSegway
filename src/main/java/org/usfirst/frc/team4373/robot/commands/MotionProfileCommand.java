package org.usfirst.frc.team4373.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team4373.robot.commands.profiles.MotionProfile;
import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class MotionProfileCommand extends Command {

    Drivetrain drivetrain;

    Drivetrain.MotorID motorID;
    MotionProfileFeeder feeder;

    boolean initialized = false;

    /**
     * Instantiates a new MotionProfileCommand.
     * @param motorID the ID of the motor to use.
     * @param prof the profile to use.
     */
    public MotionProfileCommand(Drivetrain.MotorID motorID, MotionProfile prof) {
        requires(this.drivetrain = Drivetrain.getInstance());
        this.motorID = motorID;
        this.feeder = new MotionProfileFeeder(this.drivetrain.getTalon(this.motorID), prof);
    }

    @Override
    public void initialize() {
        System.out.println("Motion Profile Command started");
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
        return feeder.isComplete();
    }

    @Override
    protected void end() {
        System.out.println("Motion Profile Command finished");
        this.drivetrain.zeroMotors();
    }

    @Override
    protected void interrupted() {
        end();
    }
}
