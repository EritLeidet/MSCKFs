package com.example.msckfs.utils;
import static org.opencv.core.CvType.CV_64F;

import org.opencv.core.Mat;

// TODO: do the same thing with vec?
public class Matx33d {
    private static final int type = CV_64F;
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

    public static boolean isMatx33d(Mat mat) {
        return Matx.isMatx(mat, numRows, numCols, type);
    }

    public static Mat eye() {
        return Mat.eye(numRows, numCols, type);
    }

}
