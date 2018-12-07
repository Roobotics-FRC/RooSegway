package org.usfirst.frc.team4373.robot;

import org.usfirst.frc.team4373.robot.subsystems.Drivetrain;

public class CTRTalonTelemetry implements MotionProfileTelemetry {

    private Drivetrain drivetrain;
    private TalonTelemetyPolarity polarity;
    public enum TalonTelemetyPolarity {
        LeftSide, RightSide
    }

    public CTRTalonTelemetry(TalonTelemetyPolarity pol) {
        super();
        this.drivetrain = Drivetrain.getInstance();
        this.polarity = pol;
    }


    @Override
    public double getCurrentVelocity() {
        switch (polarity) {
            case LeftSide: return this.drivetrain.getLeftVelocity();
            case RightSide: return this.drivetrain.get
        }
        return this.drivetrain.getLeftVelocity();
    }

    @Override
    public long getCurrentTimeMS() {
        return 0;
    }

    @Override
    public long getCurrentRotations() {
        return 0;
    }
}
