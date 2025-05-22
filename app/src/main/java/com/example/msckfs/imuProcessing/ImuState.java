package com.example.msckfs.imuProcessing;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class ImuState {

    public final RealVector
            orientation,
            velocity,
            position,
            gyroBias,
            acc_bias,
            orientation_null;

    public final RealMatrix
            gyroNoise,
            gyroBiasNoise,
            accNoise,
            accBiasNoise;

    public final double timestamp; // unix time

    public ImuState() {
        velocity = new ArrayRealVector(3, 0); // vector [0,0,0]

    }
}
