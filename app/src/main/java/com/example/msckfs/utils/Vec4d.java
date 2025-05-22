package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

// TODO
public class Vec4d extends MatOfDouble {
    public Vec4d() {
        super(0d,0d,0d,0d);
    }

    public Vec4d(double x, double y, double z, double w) {
        super(x,y,z,w);
    }
    public double get(int index) {
        if (index > 3 || index < 0 ) throw new IllegalArgumentException("Vec4d access index must be between 0 and 2");
        return super.at(double.class, index, 0).getV();
    }
}
