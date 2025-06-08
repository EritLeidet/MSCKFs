package com.example.msckfswin.utils;


import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;

public class MathUtils {

    // TODO: copy unit tests

    // TODO: implementations

    public static RealMatrix skewSymmetric(RealVector mat) {
        return null; //TODO
    }

    public static Mat quaternionToRotation(Mat mat) {
        return null; //TODO
    }

    public static void quaternionNormalize(Mat mat) {}  // TODO

    public static MatOfDouble rotationToQuaternion(Mat mat) {return null;} // TODO

    /*
     *  @brief Create a skew-symmetric matrix from a 3-element vector.
     *  @note Performs the operation:
     *  w   ->  [  0 -w3  w2]
     *          [ w3   0 -w1]
     *          [-w2  w1   0]
     */

    /*
    public static RealMatrix skewSymmetric(final RealVector w) {
        assert(w != null);
        RealMatrix out = new Array2DRowRealMatrix(3,3);
        out.setEntry(0,0,0.0);
        out.setEntry(0,1,-w.getEntry(2));
        out.setEntry(0,2,w.getEntry(1));
        out.setEntry(1,0,w.getEntry(2));
        out.setEntry(0,2,w.getEntry(1));
        return out; //TODO
    }

     */

    /**
    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w) {
        Eigen::Matrix3d w_hat;
        w_hat(0, 0) = 0;
        w_hat(0, 1) = -w(2);
        w_hat(0, 2) = w(1);
        w_hat(1, 0) = w(2);
        w_hat(1, 1) = 0;
        w_hat(1, 2) = -w(0);
        w_hat(2, 0) = -w(1);
        w_hat(2, 1) = w(0);
        w_hat(2, 2) = 0;
        return w_hat;
    }
     **/

    /*
    public static RealMatrix quaternionToRotation(RealVector q) {
        RealMatrix out = new Array2DRowRealMatrix(3,3);

        // return 3D vector
        return out; //TODO

    }

     */


}
