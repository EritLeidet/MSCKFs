package com.example.msckfs.utils;

import org.opencv.core.MatOfDouble;

public class Vec3d extends MatOfDouble {

    public Vec3d() {
        super(0d,0d,0d);
    }

    public Vec3d(double x, double y, double z) {
        super(x,y,z);
    }

    public double get(int index) {
        if (index > 2 || index < 0 ) throw new IllegalArgumentException("Vec3d access index must be between 0 and 2");
        return super.at(double.class, index, 0).getV();
    }
}
