package com.example.msckfswin.imuProcessing;

import com.example.msckfswin.utils.Vec3f;

import org.opencv.core.MatOfFloat;

public class ImuMessage {

    // public final Object orientationCovariance;
    public final MatOfFloat angularVelocity; // vec3
    // public final Object linearAcceleration;

    public final long timestamp; // unix time

    public ImuMessage(long timestamp, MatOfFloat angularVelocity) {
        this.angularVelocity = angularVelocity;
        this.timestamp = timestamp;
    }
}
