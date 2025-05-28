package com.example.msckfs.utils;

import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;


// TODO: should I refactor it to work like Matx33d?
public abstract class Vecd extends MatOfDouble {
    public Vecd(int vecLength) {
        super(new double[vecLength]);
    }

    public Vecd(int vecLength, Mat mat) {
        super(mat);
        if (mat.rows() != vecLength) throw new IllegalArgumentException("Input mat must be same size as vector.");

    }



    public Vecd(int vecLength, double... data) {
        super(data);
        if (data.length != vecLength) throw new IllegalArgumentException("Data array must be same size as vector.");

    }


    public double get(int index) {
        return super.at(double.class, index, 0).getV();
    }

}
