package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

public class Vec3f extends Vecf {

    private static final int vecLength = 3;
    public Vec3f() {
        super(vecLength);
    }

    public Vec3f(float x, float y, float z) {
        super(vecLength, x,y,z);
    }

    public Vec3f(Mat mat) {
        super(vecLength, mat);
    }
}