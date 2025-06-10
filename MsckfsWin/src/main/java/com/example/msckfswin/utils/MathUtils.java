package com.example.msckfswin.utils;


import org.apache.commons.geometry.euclidean.threed.rotation.QuaternionRotation;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealLinearOperator;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.numbers.quaternion.Quaternion;


// Convention used in vector representation: [x, y, z, w(scalar)]^T
// Convention used by Quaternion class: [w(scalar),x,y,z]
public class MathUtils {

    // own work.
    public static RealVector quaternionToVector(Quaternion q) {
        return new ArrayRealVector(new double[]{q.getX(), q.getY(), q.getZ(), q.getScalarPart()});
    }

    // own work.
    public static Quaternion vectorToQuaternion(RealVector v) {
        assert(v.getDimension() == 4);
        return Quaternion.of(v.getEntry(3), v.getEntry(0), v.getEntry(1), v.getEntry(2));
    }

    // own work
    public static void insertColumnVector(RealMatrix m, RealVector v, int row, int col) {
        RealVector colVector = m.getColumnVector(col);
        colVector.setSubVector(row,v);
        m.setColumnVector(col, colVector);
    }

    // own work
    public static void insertRowVector(RealMatrix m, RealVector v, int row, int col) {
        RealVector rowVector = m.getRowVector(row);
        rowVector.setSubVector(col,v);
        m.setColumnVector(row, rowVector);
    }


    // TODO: copy unit tests

    public static RealMatrix skewSymmetric(RealVector mat) {
        return null; //TODO
    }


    /*
    public static RealMatrix quaternionToRotation(RealVector q) {
        return null; // TODO
    }

     */
    public static RealMatrix quaternionToRotation(Quaternion q) {
        // TODO: different quaternion convention C++ / Java
        final RealVector qVec = MatrixUtils.createRealVector(new double[]{q.getX(), q.getY(), q.getZ(), q.getScalarPart()});
        final double q4 = q.getScalarPart();
        // TODO: operate instead of preMultiply?
        return MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(2*q4*q4-1).subtract(skewSymmetric(qVec).scalarMultiply(2*q4)).add(transpose(qVec).preMultiply(qVec.mapMultiply(2))); // TODO: instead of custom tranpose, convert to
    }

    // (own work) // TODO: unit test?
    public static RealMatrix transpose(RealVector vec) {
        return MatrixUtils.createRealMatrix(new double[][]{vec.toArray()}).transpose();
    }

    public static void quaternionNormalize(Quaternion mat) {}  // TODO

    public static Quaternion rotationToQuaternion(RealMatrix mat) {return null;} // TODO

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
        out.setEntry(0,2,w.getEntry(1));
        return out; //TODO
    }

     */

    /**
    inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& w) {
        Eigen::Matrix3d w_hat;
     out.setEntry(1,0,w.getEntry(2));
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
