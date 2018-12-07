package org.usfirst.frc.team4373.robot;

import java.io.IOException;

public class MotionProfileManagement {

    private MotionProfileCollector mpCollector;
    private MotionProfileCollection mpRepository;
    private MotionProfileExecutor motionProfileExecutor;

    public MotionProfileManagement(MotionProfileCollector mpCollector,
                                   MotionProfileCollection mpRepository,
                                   MotionProfileExecutor motionProfileExecutor) {
        this.mpCollector = mpCollector;
        this.mpRepository = mpRepository;
        this.motionProfileExecutor = motionProfileExecutor;
    }

    public MotionProfileCollector getCollector() {
        return this.mpCollector;
    }

    public MotionProfileCollection getCollection() {
        return this.mpRepository;
    }

    public MotionProfileExecutor getExecutor() {
        return this.motionProfileExecutor;
    }

    public void recordMotionProfile() {
        this.mpCollector.collect();
    }

    public MotionProfileCollector.MotionProfileTuple fetchMotionProfile(String mpName) {
        return this.mpRepository.get(mpName);
    }

    public void commitMotionProfile(String motionProfileName) {
        mpRepository.put(motionProfileName, this.mpCollector.getFormattedMotionProfile());
    }

    public void serializeMotionProfiles(String directoryName) {
        mpRepository.serializeMotionProfiles();
    }

    public void clearMotionProfileRecording() {
        this.mpCollector.clear();
    }

    public void executeMotionProfile() {
        this.motionProfileExecutor.execute();
    }



    public void preempt() {}//todo
}
