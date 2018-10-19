package org.usfirst.frc.team4373.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import org.usfirst.frc.team4373.robot.splines.CubicSplineFitter;
import org.usfirst.frc.team4373.robot.splines.CubicSplineInterval;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Serializable;
import java.util.ArrayList;

public class MotionProfileCollector {

    private MotionProfileTelemetry telemetry;
    private ArrayList<MotionRecordFormat> record;
    public MotionProfileCollector(MotionProfileTelemetry telemetry) {
        this.telemetry = telemetry;
        this.record = new ArrayList<>(3000);
    }

    public void collect() {
        this.record.add(new MotionRecordFormat(this.telemetry.getCurrentVelocity(),
                this.telemetry.getCurrentTimeMS(), this.telemetry.getCurrentRotations()));
    }

    public MotionRecordFormat mostRecentRecord() {
        return this.record.get(this.record.size() - 1);
    }

    public ArrayList<MotionRecordFormat> getMotionProfileRecord() {
        return this.record;
    }

    public void serializeMotionProfileRecord(OutputStreamWriter out) throws IOException {
        for (MotionRecordFormat a : this.record) {
            out.write(this.record.toString());
            out.write("\n");
        }
        out.flush();
    }

    public CubicSplineInterval[] fitV() {
        double[] x = new double[this.record.size()], a = new double[this.record.size()];
        for (int i = 0; i < x.length; ++i) {
            x[i] = this.record.get(i).timeMS;
            a[i] = this.record.get(i).velocity;
        }
        CubicSplineFitter interpolator = new CubicSplineFitter(x, a);
        interpolator.interpolate();
        return interpolator.getCurve();
    }

    public CubicSplineInterval[] fitRotations() throws IllegalArgumentException {
        double[] x = new double[this.record.size()], a = new double[this.record.size()];
        for(int i = 0; i < x.length ; ++i) {
            x[i] = this.record.get(i).timeMS;
            a[i] = this.record.get(i).rotations;
        }
        CubicSplineFitter interpolator = new CubicSplineFitter(x, a);
        interpolator.interpolate();
        return interpolator.getCurve();
    }

    public class MotionRecordFormat implements Serializable {
        double velocity;
        long timeMS;
        long rotations;

        public MotionRecordFormat(double velocity, long timeMS, long rotations) {
            this.velocity = velocity;
            this.timeMS = timeMS;
            this.rotations = rotations;
        }

        @Override
        public String toString() {
            return this.velocity + " " + this.timeMS + " " + this.rotations + ";";
        }
    }

}
