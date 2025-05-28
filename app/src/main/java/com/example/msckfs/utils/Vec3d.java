package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class Vec3d extends Vecd {

    private static final int vecLength = 3;
    public Vec3d() {
        super(vecLength);
    }

    public Vec3d(double x, double y, double z) {
        super(vecLength, x,y,z);
    }

    public Vec3d(Mat mat) {
        super(vecLength, mat);
    }
}
