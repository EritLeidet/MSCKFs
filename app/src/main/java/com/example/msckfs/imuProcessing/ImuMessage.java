package com.example.msckfs.imuProcessing;

import com.example.msckfs.utils.Vec3f;

import org.apache.commons.math3.linear.RealVector;

// TODO: DO I NEED TO CHANGE THIS CLASS IN ORDER FOR Iterator<ImuMessage> TO RETURN THINGS IN ORDER? See: iterator() doc. Check BlockedQueue doc.
public class ImuMessage {

    // TODO: change data types;
    public final Object orientationCovariance;
    public final Vec3f angularVelocity; // vec3
    public final Object linearAcceleration;

    public final long timestamp; // unix time
}
