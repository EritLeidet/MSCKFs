package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;

public abstract class Vecf extends MatOfFloat {
    public Vecf(int vecLength) {
        super(new float[vecLength]);
    }

    public Vecf(int vecLength, Mat mat) {
        super(mat);
        if (mat.rows() != vecLength) throw new IllegalArgumentException("Input mat must be same size as vector.");

    }



    public Vecf(int vecLength, float... data) {
        super(data);
        if (data.length != vecLength) throw new IllegalArgumentException("Data array must be same size as vector.");

    }


    public float get(int index) {
        return super.at(float.class, index, 0).getV();
    }


}
