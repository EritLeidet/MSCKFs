package com.example.msckfswin.imuProcessing;

import com.example.msckfswin.utils.Matx33d;
import com.example.msckfswin.utils.Vec3d;
import com.example.msckfswin.utils.Vec4d;

import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.numbers.quaternion.Quaternion;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class ImuState {

    public int id;

    public static int nextId; // TODO: warum diese extra Variabel?

    public static RealVector gravity = Vec3d.create(0,0,-9.81);
    public double timestamp; // unix time

    // TODO: do orientation, pos etc. even need an initial value? Bc. seems to be overwritten?
    public Quaternion orientation; // TODO: default val (0,0,0,1);
    public RealVector position = Vec3d.create();
    public RealVector velocity = Vec3d.create();

    public RealVector gyroBias = Vec3d.create();
    public RealVector accBias = Vec3d.create();



    public Quaternion orientationNull = Vec4d.create(0,0,0,1);
    public RealVector positionNull = Vec3d.create();
    public RealVector velocityNull = Vec3d.create();

    // Transformation between the IMU and the left camera (cam0)
    public RealMatrix rImuCam0 = Matx33d.eye(); // TODO: what value? Python is clear, but C++ looks diff?
    public RealVector tCam0Imu = Vec3d.create();


}
