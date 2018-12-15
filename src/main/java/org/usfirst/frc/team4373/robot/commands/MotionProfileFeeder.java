package org.usfirst.frc.team4373.robot.commands;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import org.usfirst.frc.team4373.robot.RobotMap;
import org.usfirst.frc.team4373.robot.commands.profiles.MotionProfile;

public class MotionProfileFeeder {

    private MotionProfileStatus status = new MotionProfileStatus();

    // For potential logging purposes
    double curPos = 0;
    double curHead = 0;
    double curVel = 0;

    double endHeading = 0;

    private WPI_TalonSRX primaryTalon;
    private MotionProfile profile;

    private int state = 0;
    private int loopTimeout = -1;
    private boolean completed = false;

    private boolean start = false;
    private boolean forward = false;

    private SetValueMotionProfile setValue = SetValueMotionProfile.Disable;
    private int minPointsInTalon = 5;
    private int numLoopsTimeout = 10;

    class PeriodicRunnable implements java.lang.Runnable {
        public void run() {
            primaryTalon.processMotionProfileBuffer();
        }
    }

    Notifier notifer = new Notifier(new PeriodicRunnable());

    /**
     * Constructs a new motion profile feeder.
     * @param motorController the motor controller on which to run the feeder.
     * @param profile the profile to run.
     */
    public MotionProfileFeeder(WPI_TalonSRX motorController, MotionProfile profile) {
        this.primaryTalon = motorController;
        this.primaryTalon.changeMotionControlFramePeriod(5);
        this.profile = profile;
        notifer.startPeriodic(0.005);
    }

    /**
     * Primary execution method of the motion profile feeder—call periodically.
     */
    public void control() {
        primaryTalon.getMotionProfileStatus(status);

        if (loopTimeout == 0) {
            DriverStation.reportError("[Motion Profile] No Progress", false);
        } else {
            --loopTimeout;
        }

        if (!isMotionProfileMode(this.primaryTalon.getControlMode())) {
            state = 0;
            loopTimeout = -1;
        } else {
            switch (state) {
                case 0:
                    if (start) {
                        start = false;
                        setValue = SetValueMotionProfile.Disable;
                        startFilling();

                        state = 1;
                        loopTimeout = numLoopsTimeout;
                    }
                    break;
                case 1:
                    if (status.btmBufferCnt > minPointsInTalon) {
                        setValue = SetValueMotionProfile.Enable;
                        state = 2;
                        loopTimeout = numLoopsTimeout;
                    }
                    break;
                case 2:
                    if (!status.isUnderrun) {
                        loopTimeout = numLoopsTimeout;
                    }
                    if (status.activePointValid && status.isLast) {
                        setValue = SetValueMotionProfile.Hold;
                        state = 0;
                        loopTimeout = -1;
                        completed = true;
                    }
                    break;
                default:
                    break;
            }
            primaryTalon.getMotionProfileStatus(status);
            curHead = primaryTalon.getActiveTrajectoryHeading();
            curPos = primaryTalon.getActiveTrajectoryPosition();
            curVel = primaryTalon.getActiveTrajectoryVelocity();
        }
    }

    /**
     * Reset the Feeder for next run-through.
     */
    public void reset() {
        this.primaryTalon.clearMotionProfileTrajectories();
        this.setValue = SetValueMotionProfile.Disable;
        this.state = 0;
        this.loopTimeout = -1;
        this.start = false;
        this.completed = false;
    }

    /**
     * Initialize the feeder.
     * @param endHeading the end heading.
     * @param forward whether it's forward.
     */
    public void start(double endHeading, boolean forward) {
        start = true;
        this.forward = forward;
        this.endHeading = endHeading;
    }

    /**
     * Returns whether the motion profile has finished executing.
     * @return whether the profile playback is done.
     */
    public boolean isComplete() {
        return this.completed;
    }

    private void startFilling() {
        startFilling(profile.points, profile.numPoints);
    }

    private void startFilling(double[][] profile, int totalCnt) {
        TrajectoryPoint point = new TrajectoryPoint();

        if (status.hasUnderrun) {
            DriverStation.reportError("[Motion Profile] Underrun", false);
            primaryTalon.clearMotionProfileHasUnderrun(RobotMap.TALON_TIMEOUT_MS);
        }

        primaryTalon.clearMotionProfileTrajectories();

        primaryTalon.configMotionProfileTrajectoryPeriod(RobotMap.MOTION_PROFILE_BASE_TRAJ_TIMEOUT,
                RobotMap.TALON_TIMEOUT_MS);

        double finalPositionRot = profile[totalCnt - 1][0];

        for (int i = 0; i < totalCnt; ++i) {
            double direction = forward ? +1 : -1;
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            double heading = endHeading * positionRot / finalPositionRot;
            point.position = direction * positionRot * RobotMap.ENCODER_UNITS_PER_ROTATION * 2;

            // Convert to units per 100 ms
            point.velocity = direction * velocityRPM * RobotMap.ENCODER_UNITS_PER_ROTATION / 600.0;
            point.auxiliaryPos = heading; /* scaled such that 3600 => 360 deg */
            point.profileSlotSelect0 = RobotMap.SLOT_MOTPROF;
            point.profileSlotSelect1 = RobotMap.SLOT_TURNING;
            point.timeDur = getTrajectoryDuration((int)profile[i][2]);
            point.zeroPos = false;
            if (i == 0) point.zeroPos = true;

            point.isLastPoint = false;
            if ((i + 1) == totalCnt) point.isLastPoint = true;

            primaryTalon.pushMotionProfileTrajectory(point);
        }
    }

    private TrajectoryDuration getTrajectoryDuration(int durationMs) {
        TrajectoryDuration retVal = TrajectoryDuration.Trajectory_Duration_0ms;
        retVal = retVal.valueOf(durationMs);
        if (retVal.value != durationMs) {
            DriverStation.reportError("Trajectory Duration not supported -"
                    + "use configMotionProfileTrajectoryPeriod instead", false);
        }
        return retVal;
    }

    private boolean isMotionProfileMode(ControlMode controlMode) {
        return controlMode == ControlMode.MotionProfile
                || controlMode == ControlMode.MotionProfileArc;
    }

    protected SetValueMotionProfile getSetValue() {
        return setValue;
    }
}
