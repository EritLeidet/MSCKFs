package com.example.msckfswin.utils;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Simplify OpenCV Mat operations
 */
public class MatOperations {

    private final Mat wrappedMat;
    public MatOperations(Mat mat) {
        this.wrappedMat = mat;
    }

    public Mat matMul(Mat inputMat) {
        Core.multiply(wrappedMat, inputMat, wrappedMat);
        return wrappedMat; // TODO: is that okay, or need to create new Mat object?
    }

    public Mat add(Mat inputMat) {
        Core.add(wrappedMat, inputMat, wrappedMat);
        return wrappedMat;
    }

    public Mat sub(Mat inputMat) {
        Core.subtract(wrappedMat, inputMat, wrappedMat);
        return wrappedMat;
    }

    public Mat div(double divisor) {
        Core.divide(wrappedMat, new Scalar(divisor), wrappedMat);
        return wrappedMat;
    }

    public Mat mult(double factor) {
        Core.multiply(wrappedMat, new Scalar(factor), wrappedMat);
        return wrappedMat;
    }

    public Mat neg() {
        return mult(-1);
    }

    public static Mat mult(Mat src, double factor) {
        Mat dest = new Mat();
        Core.multiply(src, new Scalar(factor), dest);
        return dest;
    }

    public static Mat mult(double factor, Mat src) {
        return mult(src, factor);
    }

    public static Mat add(Mat src1, Mat src2) {
        Mat dest = new Mat();
        Core.add(src1, src2, dest);
        return dest;
    }

    public static Mat neg(Mat mat) {
        Mat dest = new Mat();
        Core.multiply(mat, new Scalar(-1), dest);
        return dest;
    }

    public static Mat sub(Mat src1, Mat src2) {
        Mat dest = new Mat();
        Core.subtract(src1, src2, dest);
        return dest;
    }

    public static Mat div(Mat dividend, double divisor) {
        Mat dest = new Mat();
        Core.divide(dividend,new Scalar(divisor),dest);
        return dest;
    }


    // TODO: maybe test?
    public static Mat add(Mat... mats) {
        Mat dest = new Mat();
        mats[0].copyTo(dest);
        for (int i = 1; i < mats.length; i++) {
            Core.add(dest, mats[i], dest);
        }

        return dest;
    }



}
