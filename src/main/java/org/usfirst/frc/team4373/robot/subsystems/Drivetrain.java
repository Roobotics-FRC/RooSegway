package org.usfirst.frc.team4373.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A programmatic representation of the robot's drivetrain
 *
 * @author Samasaur
 */
public class Drivetrain extends Subsystem {

    //declare variables

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        return instance == null ? instance = new Drivetrain() : instance;
    }

    private Drivetrain() {
        //initialize variables
    }

    //setting methods go here

    //getting methods go here

    @Override
    protected void initDefaultCommand() {
//        setDefaultCommand(new BalancingCommand()); //it's a balancing act
    }
}
