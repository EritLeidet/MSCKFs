package com.example.msckfswin;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.complex.*;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.numbers.quaternion.Quaternion;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class CamState {

    final public int id;


    // TODO: could you make final and in constructor?
    public long timestamp;
    public Quaternion orientation; // vec4d
    public RealVector position; // vec3d

    org.apache.commons.numbers.quaternion.Quaternion orientationNull;
    RealVector positionNull;

    public CamState(int id) {
        this.id = id;
    }
}
