package com.example.msckfswin.utils;

import static org.opencv.core.CvType.CV_32F;

import org.opencv.core.Mat;

public abstract class Matx33f {


    private static final int type = CV_32F;
    private static final int numRows = 3;
    private static final int numCols = 3;

    /**
     * Fills with zeroes.
     */
    public static Mat create() {
        return new Mat(numRows,numCols,type);
    }

    public static Mat create(double[] data) {
        return Matx.create(numRows,numCols,type,data);
    }

    public static boolean isMatx33f(Mat mat) {
        return Matx.isMatx(mat, numRows, numCols, type);
    }

    public static float get(Mat mat, int row, int col) {
        return mat.at(float.class, row, col).getV();
    }
    public static Mat eye() {
        return Mat.eye(numRows, numCols, type);
    }
}
