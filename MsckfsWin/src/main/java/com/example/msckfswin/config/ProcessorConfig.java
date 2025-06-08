package com.example.msckfswin.config;

import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;

/**
 * Taken from Python Impl//TODO: how 2 give credit
 */
public class ProcessorConfig {

    // TODO: how to config for Android?

     public final int gridRow = 4;
    public final int gridCol = 4;
    public final int gridMinFeatureNum = 2;
    public final int gridMaxFeatureNum = 4; // TODO: in Python = 5


    public final int pyramidLevels = 3;
    public final int patchSize = 31;
    public final int fastThreshold = 20;

    public final int maxIteration = 30;

    public final double trackPrecision = 0.01;

    public final double ransacThreshold = 3;


    //calibration parameters // TODO: MatOfDouble or MatOfFloat?
    public MatOfInt cam0Resolution;// vec2
    public MatOfDouble cam0Intrinsics; // vec4
    public String cam0DistortionModel;
    public MatOfDouble cam0DistortionCoeffs; // vec4

    private ProcessorConfig() {
    }

    public static ProcessorConfig getEuRoCconfig() {
        ProcessorConfig config = new ProcessorConfig();
        config.cam0DistortionModel = "radtan";
        config.cam0Resolution = new MatOfInt(752, 480);
        config.cam0Intrinsics = new MatOfDouble(458.654d, 457.296d, 367.215d, 248.375d);
        config.cam0DistortionCoeffs = new MatOfDouble(-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);
        return config;
    }
}
