package com.example.msckfswin.imuProcessing;

import com.example.msckfswin.utils.Vec3f;

import org.apache.commons.math3.linear.RealVector;
import org.opencv.core.MatOfFloat;

public class ImuMessage {

    // TODO: used to be MatOfFloat, but changed bc. MSCKF Matrix type
    public final RealVector angularVelocity; // vec3
    public final RealVector linearAcceleration; // vec3

    public final long timestamp; // unix time

    public ImuMessage(long timestamp, MatOfFloat angularVelocity) {
        this.angularVelocity = angularVelocity;
        this.timestamp = timestamp;
    }
}
