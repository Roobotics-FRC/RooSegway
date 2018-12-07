package org.usfirst.frc.team4373.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc.team4373.robot.splines.CubicSplineFitter;
import org.usfirst.frc.team4373.robot.splines.CubicSplineInterval;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Serializable;
import java.util.ArrayList;

/**
 * A collector class for collecting motion profiles
 */
public class MotionProfileCollector {

    private MotionProfileTelemetry telemetry;
    private ArrayList<MotionRecordFormat> record;

    /**
     * Constructor method
     *
     * @param telemetry A class implementing methods specified in
     *                  the telemetry interface.
     */
    public MotionProfileCollector(MotionProfileTelemetry telemetry) {
        this.telemetry = telemetry;
        this.record = new ArrayList<>(3000);
    }

    /**
     * Can get called iteratively; collects a single entry of telemetry data
     * and stores it inside our record
     */
    public void collect() {
        this.record.add(new MotionRecordFormat(this.telemetry.getCurrentVelocity(),
                this.telemetry.getCurrentTimeMS(), this.telemetry.getCurrentRotations()));
    }

    /**
     * Returns the most recent recorded record
     *
     * @return a MotionRecordFormat instance
     */
    public MotionRecordFormat mostRecentRecord() {
        return this.record.get(this.record.size() - 1);
    }

    /**
     * Returns all recorded datat in ArrayList format;
     * the record entries are naturally sorted in temporal order
     *
     * @return an ArrayList instance with all records
     */
    public ArrayList<MotionRecordFormat> getMotionProfileRecord() {
        return this.record;
    }

    /**
     * Serializes all records
     *
     * @param out An output source
     * @throws IOException if write fails
     */
    public void serializeMotionProfileRecord(OutputStreamWriter out)
            throws IOException,
            IllegalArgumentException {

        CubicSplineInterval[] velocityIntervals = this.fitV();
        CubicSplineInterval[] rotationIntervals = this.fitRotations();
        out.write("; Auto-generated motion profile");
        out.write("; Telemetry Points Below");
        for (MotionRecordFormat a : this.record) {
            out.write(this.record.toString());
            out.write("\n");
        }
        out.write("; v/t fitting Below");
        for (CubicSplineInterval intr : velocityIntervals) {
            out.write(intr.toString());
        }
        out.write("; rotation/t fitting below");
        for (CubicSplineInterval intr : rotationIntervals) {
            out.write(intr.toString());
        }
        out.write("; end of file");
        out.flush();

    }

    /**
     * fits V axis to T axis
     *
     * @return a valid spline on an interval
     * @throws IllegalArgumentException when fitting fails
     */
    public CubicSplineInterval[] fitV() throws IllegalArgumentException {
        double[] x = new double[this.record.size()], a = new double[this.record.size()];
        for (int i = 0; i < x.length; ++i) {
            x[i] = this.record.get(i).timeMS;
            a[i] = this.record.get(i).velocity;
        }
        CubicSplineFitter interpolator = new CubicSplineFitter(x, a);
        interpolator.interpolate();
        return interpolator.getCurve();
    }

    /**
     * Fits NumRotations to T axis
     *
     * @return a valid Spline interval
     * @throws IllegalArgumentException when fitting fails
     */
    public CubicSplineInterval[] fitRotations() throws IllegalArgumentException {
        double[] x = new double[this.record.size()], a = new double[this.record.size()];
        for (int i = 0; i < x.length; ++i) {
            x[i] = this.record.get(i).timeMS;
            a[i] = this.record.get(i).rotations;
        }
        CubicSplineFitter interpolator = new CubicSplineFitter(x, a);
        interpolator.interpolate();
        return interpolator.getCurve();
    }

    /**
     * Gets a formatted motion profile
     * @return
     */
    public MotionProfileTuple getFormattedMotionProfile() {
        return new MotionProfileTuple((MotionRecordFormat[]) this.getMotionProfileRecord().toArray(),
                this.fitV(), this.fitRotations());
    }

    void clear() {
        this.record.clear();
    }

    /**
     * Represents a single motion record
     */
    public class MotionRecordFormat implements Serializable {
        double velocity;
        long timeMS;
        long rotations;

        /**
         * Constructor for the motion record
         *
         * @param velocity  velocity from telemetry source
         * @param timeMS    time duration from telemetry source
         * @param rotations number of rotations from telemetry source
         */
        public MotionRecordFormat(double velocity, long timeMS, long rotations) {
            this.velocity = velocity;
            this.timeMS = timeMS;
            this.rotations = rotations;
        }

        /**
         * Serializes the data into a String format for later archive
         *
         * @return representation of MotionRecordFormat as a string
         */
        @Override
        public String toString() {
            return this.velocity + ":" + this.timeMS + ":" + this.rotations;
        }
    }

    public class MotionProfileTuple implements Serializable {
        public MotionRecordFormat[] motionRecords;
        public CubicSplineInterval[] vSpline;
        public CubicSplineInterval[] tSpline;

        public MotionProfileTuple(MotionRecordFormat[] motionRecords,
                                  CubicSplineInterval[] vSpline,
                                  CubicSplineInterval[] tSpline) {
            this.motionRecords = motionRecords;
            this.vSpline = vSpline;
            this.tSpline = tSpline;
        }
    }
}
