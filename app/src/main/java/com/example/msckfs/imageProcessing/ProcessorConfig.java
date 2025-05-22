package com.example.msckfs.imageProcessing;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

/**
 * Taken from Python Impl//TODO: how 2 give credit
 */
public class ProcessorConfig {

    // TODO: how to config for Android?
    public final int grid_row;
    public final int grid_col;
    public final int grid_min_feature_num;
    public final int grid_max_feature_num;


    public final int pyramid_levels;
    public final int patch_size;
    public final int fast_threshold;

    public final int max_iteration;

    public final double track_precision;

    public final double ransac_threshold;

    public final double ransac_threshold;

    //calibration parameters
    public final RealMatrix TImuCam0; // TODO: In Python 4x4, but C++ 3x3?
    public final RealVector cam0Resolution// vec2
    public final RealVector cam0Intrinsics; // vec4 // TODO: muss Vector Datentyp zum Rechnen verändert werden?
    public final String cam0DistortionModel;
    public final RealVector cam0DistortionCoeffs; // vec4

    public ProcessorConfig(String yamlPath) {

    }
}
