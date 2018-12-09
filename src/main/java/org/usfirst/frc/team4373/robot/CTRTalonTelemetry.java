package org.usfirst.frc.team4373.robot;

import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class CTRTalonTelemetry implements MotionProfileTelemetry {

    private Drivetrain drivetrain;
    private Drivetrain.MotorID motorID;
    private int pidIdx;

    public CTRTalonTelemetry(Drivetrain.MotorID motorID, int pidIdx) {
        super();
        this.drivetrain = Drivetrain.getInstance();
        this.motorID = motorID;
        this.pidIdx = pidIdx;
    }


    @Override
    public double getCurrentVelocity() {
        return drivetrain.getSensorVelocity(motorID, pidIdx);
    }

    @Override
    public long getCurrentTimeMS() {
        return System.currentTimeMillis();
    }

    @Override
    public long getCurrentRotations() {
        return this.drivetrain.getSensorPosition(motorID, pidIdx);
    }
}
