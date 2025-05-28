package com.example.msckfs.utils;

import org.opencv.core.Mat;

// TODO
public class Vec4d extends Vecd {

    private static final int vecLength = 4;
    public Vec4d() {
        super(vecLength);
    }

    public Vec4d(double x, double y, double z, double w) {
        super(vecLength, x,y,z,w);
    }

    public Vec4d(Mat mat) {
        super(vecLength, mat);
    }
}
