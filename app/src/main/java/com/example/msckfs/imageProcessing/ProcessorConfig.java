package com.example.msckfs.imageProcessing;

import com.example.msckfs.utils.Vec2d;
import com.example.msckfs.utils.Vec4d;

import org.apache.commons.math3.linear.RealMatrix;

/**
 * Taken from Python Impl//TODO: how 2 give credit
 */
public class ProcessorConfig {

    // TODO: how to config for Android?
    public final int gridRow;
    public final int gridCol;
    public final int gridMinFeatureNum;
    public final int gridMaxFeatureNum;


    public final int pyramid_levels;
    public final int patch_size;
    public final int fast_threshold;

    public final int max_iteration;

    public final double track_precision;

    public final double ransac_threshold;

    public final double ransac_threshold;

    //calibration parameters
    public final RealMatrix TImuCam0; // TODO: In Python 4x4, but C++ 3x3?
    public final Vec2d cam0Resolution// vec2
    public final Vec4d cam0Intrinsics; // vec4 // TODO: 4d oder 4f?
    public final String cam0DistortionModel;
    public final Vec4d cam0DistortionCoeffs; // vec4

    public ProcessorConfig(String yamlPath) {

    }
}
