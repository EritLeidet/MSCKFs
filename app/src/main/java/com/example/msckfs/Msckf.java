package com.example.msckfs;

import static com.example.msckfs.utils.MathUtils.*;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import com.example.msckfs.imuProcessing.ImuState;

public class Msckf {

    private StateServer stateServer;

    // TODO: how would you convert a RealVector to a RealMatrix?
    public void processModel(final double time, RealVector m_gyro, RealVector m_acc) {

        // Remove the bias from the measured gyro and acceleration
        ImuState imuState = stateServer.imuState;
        RealVector gyro = m_gyro.subtract(imuState.gyroBias);
        RealVector acc = m_acc.subtract(imuState.acc_bias);
        double dtime = time - imuState.timestamp;
        // Compute discrete transition and noise covariance matrix
        RealMatrix F = new Array2DRowRealMatrix(21, 21);
        RealMatrix G = new Array2DRowRealMatrix(21, 12);

        //TODO: how to access part of a matrix?
        //TODO: code implies that gyro/acc is 3x3, although should be 3x1?
        // Attention: Submatrix, enter STARTING position

        //TODO: should negation apply to both row and col?
        F.setSubMatrix(
                skewSymmetric(gyro)
                .scalarMultiply(-1)
                .getData(),0,0);
        F.setSubMatrix(MatrixUtils
                        .createRealIdentityMatrix(3)
                        .scalarMultiply(-1)
                        .getData(),0,3);

        //TODO: should the multiplication here be scalar or dot?
        //TODO: should negation apply before or after dot multiplication?
        F.setSubMatrix(
                quaternionToRotation(imuState.orientation)
                        .transpose()
                        .multiply(skewSymmetric(acc))
                        .scalarMultiply(-1)
                        .getData(), 6,0);
        F.setSubMatrix(
                quaternionToRotation(imuState.orientation)
                        .transpose()
                        .scalarMultiply(-1)
                        .getData(),6,9);
        F.setSubMatrix(MatrixUtils
                .createRealIdentityMatrix(3)
                .getData(),12,6);


        G.setSubMatrix(MatrixUtils
                .createRealIdentityMatrix(3)
                .scalarMultiply(-1)
                .getData(),0,0);
        G.setSubMatrix(MatrixUtils
                .createRealIdentityMatrix(3)
                .getData(),3,3);
        G.setSubMatrix(
                quaternionToRotation(imuState.orientation)
                        .transpose()
                        .scalarMultiply(-1)
                        .getData(),6,6);
        G.setSubMatrix(MatrixUtils
                .createRealIdentityMatrix(3)
                .getData(),9,9);

        // Approximate matrix exponential to the 3rd order,
        // which can be considered to be accurate enough assuming
        // dtime is within 0.01s.
        RealMatrix Fdt = F.scalarMultiply(dtime); //TODO: does this change F?
        //TODO

        // Propogate the state using 4th order Runge-Kutta
        predictNewState(dtime, gyro, acc);

        // Modify the transition matrix
        RealMatrix R_kk_1 = quaternionToRotation(imuState.orientation_null);

    }

    public void predictNewState(double dtime, RealVector gyro, RealVector acc) {
        //TODO: is there a mono/stereo impl merge conflict?
    }
}
