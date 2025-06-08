package com.example.msckfswin;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class CamState {

    final public int id;


    // TODO: could you make final and in constructor?
    public long timestamp;
    public RealVector orientation; // vec4d
    public RealVector position; // vec3d

    MatOfDouble orientationNull;
    MatOfDouble positionNull;

    public CamState(int id) {
        this.id = id;
    }
}
