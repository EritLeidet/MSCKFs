package com.example.msckfswin;

import static com.example.msckfswin.utils.MatOperations.add;
import static com.example.msckfswin.utils.MatOperations.div;
import static com.example.msckfswin.utils.MatOperations.mult;
import static com.example.msckfswin.utils.MatOperations.neg;
import static com.example.msckfswin.utils.MatOperations.sub;
import static com.example.msckfswin.utils.MathUtils.*;
import static com.example.msckfswin.utils.Matx.setD;

import static org.opencv.core.CvType.CV_64F;

import com.example.msckfswin.imageProcessing.Feature;
import com.example.msckfswin.imageProcessing.FeatureMeasurement;
import com.example.msckfswin.imageProcessing.FeatureMessage;
import com.example.msckfswin.imuProcessing.ImuMessage;
import com.example.msckfswin.imuProcessing.ImuState;
import com.example.msckfswin.utils.MatOperations;
import com.example.msckfswin.utils.MathUtils;
import com.example.msckfswin.utils.Matx;
import com.example.msckfswin.utils.Matx33d;
import com.example.msckfswin.utils.Vec2d;
import com.example.msckfswin.utils.Vec3d;
import com.example.msckfswin.utils.Vec4d;
import com.example.msckfswin.utils.Vecd;


import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class Msckf {

    // State vector
    private StateServer stateServer;

    // Features used
    private Map<Integer, Feature> mapServer;

    // IMU data buffer
    // This is buffer is used to handle the unsynchronization or
    // transfer delay between IMU and Image messages.
    private final List<ImuMessage> imuMsgBuffer = Collections.synchronizedList(new ArrayList<>()); // TODO: should be same as in ImageProcessor



    // TODO: in Konstruktor or nah?
    private boolean isGravitySet = false;
    private boolean isFirstImage = true;

    public void featureCallback(final FeatureMessage msg) {
        // Return if the gravity vector has not been set.
        if (!isGravitySet) return;

        // Start the system if the first image is received.
        // The frame where the first image is received will be
        // the origin.
        if (isFirstImage) {
            isFirstImage = false;
            stateServer.imuState.timestamp = msg.timestamp;
        }


        // double maxProcessingTime = 0.0; TODO: Lots of Logging from now on

        // Propogate the IMU state.
        // that are received before the image msg.
        batchImuProcessing(); // TODO

        // Augment the state vector.
        stateAugmentation();

        // Add new observations for existing features or new
        // features in the map server.
        addFeatureObservations();


        // Perform measurement update if necessary.
        removeLostFeatures();


        pruneCamStateBuffer();

        // Publish the odometry.
        publish();

        // Reset the system if necessary.
        onlineReset();



    }

    public void batchImuProcessing(final double timeBound) {
        // Counter how many IMU msgs in the buffer are used.
        int usedImuMsgCntr = 0;

        double imuTime; // TODO: why double?
        for (ImuMessage imuMsg: imuMsgBuffer) {
            imuTime = imuMsg.timestamp;
            if (imuTime < stateServer.imuState.timestamp) {
                usedImuMsgCntr++;
                continue;
            }
            if (imuTime > timeBound) break;

            RealVector mGyro = imuMsg.angularVelocity;
            RealVector mAcc = imuMsg.linearAcceleration;

            // Execute process model.
            processModel();
            usedImuMsgCntr++;
        }

        // Set the state ID for the new IMU state.
        stateServer.imuState.id = ImuState.nextId++;

        // Remove all used IMU msgs.
        imuMsgBuffer.subList(0, usedImuMsgCntr).clear();


    }

    public void processModel(final double timestamp, final RealVector mGyro, final RealVector mAcc) { // TODO: maybe change double to long bc. timestamp?


        // Remove the bias from the measured gyro and acceleration
        ImuState imuState = stateServer.imuState;
        RealVector gyro = mGyro.subtract(imuState.gyroBias);
        RealVector acc = mAcc.subtract(imuState.accBias);

        double dtime = timestamp - imuState.timestamp; // TODO: maybe save as a new Scalar?

        // Compute discrete transition and noise covariance matrix
        RealMatrix F = MatrixUtils.createRealMatrix(21,21);
        RealMatrix G = MatrixUtils.createRealMatrix(21,12);

        // F.block
        F.setSubMatrix(skewSymmetric(gyro).scalarMultiply(-1).getData(), 0,0);
        F.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(-1).getData(),0,3);
        F.setSubMatrix(quaternionToRotation(imuState.orientation).transpose().multiply(skewSymmetric(acc)).scalarMultiply(-1).getData(),6,0);
        F.setSubMatrix(quaternionToRotation(imuState.orientation).transpose().scalarMultiply(-1).getData(),6,9);
        F.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).getData(),12,6);

        // G.block
        G.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(-1).getData(),0,0);
        G.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).getData(),3,3);
        G.setSubMatrix(quaternionToRotation(imuState.orientation).transpose().scalarMultiply(-1).getData(),6,6);
        G.setSubMatrix(MatrixUtils.createRealIdentityMatrix(3).getData(),9,9);

        // Approximate matrix exponential to the 3rd order,
        // which can be considered to be accurate enough assuming
        // dtime is within 0.01s.
        RealMatrix Fdt = F.scalarMultiply(dtime);
        RealMatrix FdtSquare = Fdt.multiply(Fdt);
        RealMatrix FdtCube = FdtSquare.multiply(Fdt);
        RealMatrix Phi = MatrixUtils.createRealIdentityMatrix(21).add(Fdt).add(FdtSquare.scalarMultiply(0.5)).add(FdtCube.scalarMultiply(1.0/6.0)).

        // Propogate the state using 4th order Runge-Kutta
        predictNewState(dtime, gyro, acc);

        // Modify the transition matrix
        RealMatrix Rkk1 = quaternionToRotation(imuState.orientationNull);
        Phi.setSubMatrix(quaternionToRotation(ImuState.or)0,0);

        RealVector u = Rkk1.operate(ImuState.gravity);
        RealVector s = u.mapDivide( u.dotProduct(u));// Rowvector // TODO: be careful, it's a row vector?

        RealMatrix A1 = Phi.getSubMatrix(6,8,0,2);
        assert(A1.isSquare() && A1.getRowDimension() == 3);
        RealVector w1 = skewSymmetric(imuState.velocityNull.subtract(imuState.velocity)).operate(ImuState.gravity);
        Phi.setSubMatrix(A1.subtract(A1.operate(u).subtract(w1).outerProduct(s)).getData(),6,0);

        RealMatrix A2 = Phi.getSubMatrix(12,14,0,2);
        assert(A2.isSquare() && A2.getRowDimension() == 3);
        RealVector w2 = skewSymmetric(imuState.velocityNull.mapMultiply(dtime).add(imuState.positionNull).subtract(imuState.position)).operate(ImuState.gravity);
        Phi.setSubMatrix(A2.subtract(A2.operate(u).subtract(w2).outerProduct(s)).getData(),12,0);

        // Propogate the state covariance matrix.
        RealMatrix Q = Phi.multiply(G).multiply(stateServer.continuousNoiseCov).multiply(G.transpose()).multiply(Phi.transpose()).scalarMultiply(dtime);


        Mat Q = Phi.matMul(G).matMul(stateServer.continuousNoiseCov).matMul(G.t()).matMul(Phi.t());
        Core.multiply(Q,new Scalar(dtime), Q);
        assert(Q.cols() == 21 && Q.rows() == 21);
        // TODO: submat without rect?
        submat = Phi.matMul(stateServer.stateCov.submat(new Rect(0,0,21,21))).matMul(Phi.t());
        Core.add(submat, Q, submat);
        submat.copyTo(stateServer.stateCov.submat(new Rect(0,0,21,21)));

        if (!stateServer.camStates.isEmpty()) {
            // TODO: why does the block work differently now? Look how in Python
            // TODO: ...
        }

        Mat stateCovFixed = new Mat();
        Core.add(stateServer.stateCov, stateServer.stateCov.t(), stateCovFixed);
        Core.divide(stateCovFixed, new Scalar(2.0), stateCovFixed);
        stateServer.stateCov = stateCovFixed;

        // Update the state correspondes to null space.
        imuState.orientationNull = imuState.orientation;
        imuState.positionNull = imuState.position;
        imuState.velocityNull = imuState.velocity;

        // Update the state info
        stateServer.imuState.timestamp = timestamp;




    }

    public void predictNewState(final double dt, final Mat gyro, final Mat acc) {
        assert(Vec3d.isVec3d(gyro));
        assert(Vec3d.isVec3d(acc));
        Mat tempMat;

        double gyroNorm = gyro.norm(); //TODO
        Mat Omega = Mat.eye(4,4, CV_64F);
        tempMat = skewSymmetric(gyro);
        Core.multiply(tempMat, new Scalar(-1), tempMat);
        tempMat.copyTo(Omega.submat(new Rect(0,0,3,3)));
        gyro.copyTo(Omega.submat(new Rect(3,0,T,T))); // TODO
        Core.multiply(gyro, new Scalar(-1),tempMat);
        tempMat.copyTo(Omega.submat(new Rect(0,3,T,T)));

        Mat q = stateServer.imuState.orientation;
        Mat v = stateServer.imuState.velocity;
        Mat p = stateServer.imuState.position;

        // Some pre-calculation
        Mat dqDt, dqDt2;
        if (gyroNorm > 1e-5) {
            dqDt = new Mat();
            Core.multiply(Mat.eye(4,4,CV_64F), new Scalar(Math.cos(gyroNorm*dt*0.5)), dqDt);
            Core.multiply(Omega, new Scalar(1 / gyroNorm * Math.sin(gyroNorm*dt*0.5)), tempMat);
            Core.add(dqDt, tempMat, dqDt);
            dqDt = dqDt.matMul(q);

            dqDt2 = new Mat();
            Core.multiply(Mat.eye(4,4,CV_64F), new Scalar(Math.cos(gyroNorm*dt*0.25)), dqDt2);
            Core.multiply(Omega, new Scalar(1 / gyroNorm * Math.sin(gyroNorm*dt*0.25)), tempMat);
            Core.add(dqDt2, tempMat, dqDt2);
            dqDt2 = dqDt2.matMul(q);


        } else {
            dqDt = new Mat();
            Core.multiply(Omega, new Scalar(0.5*dt), dqDt);
            Core.add(Mat.eye(4,4, CV_64F), dqDt, dqDt);
            Core.multiply(dqDt, new Scalar(gyroNorm*dt*0.5), dqDt);
            dqDt = dqDt.matMul(q);

            dqDt2 = new Mat();
            Core.multiply(Omega, new Scalar(0.25*dt), dqDt2);
            Core.add(Mat.eye(4,4, CV_64F), dqDt2, dqDt2);
            Core.multiply(dqDt2, new Scalar(gyroNorm*dt*0.25), dqDt2);
            dqDt2 = dqDt2.matMul(q);
        }

        assert(Vec4d.isVec4d(dqDt));
        assert(Vec4d.isVec4d(dqDt2));

        Mat dRDtTranspose = quaternionToRotation(dqDt).t();
        Mat dRDt2Transpose = quaternionToRotation(dqDt2).t();

        // k1 = f(tn, yn)
        Mat k1vDot = quaternionToRotation(q).t().matMul(acc);
        Core.add(k1vDot,ImuState.gravity,k1vDot);
        Mat k1pDot = v;

        // k2 = f(tn+dt/2, yn+k1*dt/2)
        Mat k1v = new Mat();
        Core.multiply(k1vDot, new Scalar(dt), k1v);
        Core.divide(k1v, new Scalar(2), k1v); // TODO: error
        Core.add(v,k1v,k1v);

        Mat k2vDot = dRDt2Transpose.matMul(acc);
        k2vDot = k2vDot.matMul(ImuState.gravity);

        Mat k2pDot = k1v;

        // k3 = f(tn+dt/2, yn+k2*dt/2)
        Mat k2v = new Mat();
        Core.multiply(k2vDot, new Scalar(dt), k2v);
        Core.divide(k2v, new Scalar(2), k2v);
        Core.add(v,k2v,k2v);

        Mat k3vDot = dRDt2Transpose.matMul(acc);
        Core.add(k3vDot, ImuState.gravity, k3vDot);

        Mat k3pDot = k2v;

        // k4 = f(tn+dt, yn+k3*dt)
        Mat k3v = new Mat();
        Core.multiply(k3vDot, new Scalar(dt), k3v);
        Core.add(k3v, v, k3v);

        Mat k4vDot = dRDtTranspose.matMul(acc);
        Core.add(k4vDot, ImuState.gravity, k4vDot);

        Mat k4pDot = k3v;

        // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
        q = dqDt;
        quaternionNormalize(q);

        add(v, mult(dt/6, add(k1vDot, mult(2, k2vDot), mult(2, k3vDot), k4vDot)));
        add(p, mult(dt/6, add(k1pDot, mult(2, k2pDot), mult(2, k3pDot), k4pDot)));


    }

    public void stateAugmentation(final long timestamp) { // TODO: OG datatype is double, but timestamps are long in java. Which better?
        final Mat Ric = stateServer.imuState.rImuCam0;
        final MatOfDouble tci = stateServer.imuState.tCam0Imu;

        // Add a new camera state to the state server.
        Mat Rwi = quaternionToRotation(stateServer.imuState.orientation);
        Mat Rwc = Ric.matMul(Rwi);
        Mat tcw = add(stateServer.imuState.position, Rwi.t().matMul(tci));

        CamState camState = new CamState(stateServer.imuState.id);
        stateServer.camStates.put(stateServer.imuState.id, camState);

        camState.timestamp = timestamp;
        camState.orientation = rotationToQuaternion(Rwc);
        camState.position = new MatOfDouble(tcw);
        assert(Vec3d.isVec3d(tcw));

        camState.orientationNull = camState.orientation;
        camState.positionNull = camState.position;

        // Update the covariance matrix of the state.
        // To simplify computation, the matrix J below is the nontrivial block
        // in Equation (16) in "A Multi-State Constraint Kalman Filter for Vision
        // -aided Inertial Navigation".

        Mat J = new Mat(6,21,CV_64F);
        Ric.copyTo(J.submat(new Rect(0,0,3,3)));
        Matx33d.eye().copyTo(J.submat(new Rect(15,0, 3,3)));
        skewSymmetric(Rwi.t().matMul(tci)).copyTo(J.submat(new Rect(0,3,3,3)));
        Matx33d.eye().copyTo(J.submat(new Rect(12,3,3,3)));
        Rwi.t().copyTo(J.submat(new Rect(18,3,3,3)));

        // Resize the state covariance matrix.
        int oldRows = stateServer.stateCov.rows();
        int oldCols = stateServer.stateCov.cols();
        stateServer.stateCov.conservativeResize // TODO

        // Rename some matrix blocks for convenience.
        final Mat P11 = stateServer.stateCov.submat(new Rect(0,0,21,21));
        final Mat P12 = stateServer.stateCov.submat(new Rect()) // TODO: rect size?

        // Fill in the augmented state covariance.
        // TODO: translate the code until next comment

        // Fix the covariance to be symmetric
        Mat stateCovFixed = div(add(stateServer.stateCov, stateServer.stateCov.t()),2.0);
        stateServer.stateCov = stateCovFixed;


    }

    public void addFeatureObservations(FeatureMessage featureMsg) {
        int stateId = stateServer.imuState.id;
        int currFeatureNum = mapServer.size();
        int trackedFeaturesNum = 0;

        // Add new observations for existing features or new
        // features in the map server.
        for (final FeatureMeasurement feature : featureMsg.features) {
            if (!mapServer.containsKey(feature.id)) {
                // This is a new feature.
                Feature newFeature = new Feature(feature.id);
                newFeature.observations.put(stateId, Vec2d.create(feature.u0, feature.v0));
                mapServer.put(feature.id, newFeature);

            } else {
                // This is an old feature.
                mapServer.get(feature.id).observations.put(stateId, Vec2d.create(feature.u0, feature.v0));
                trackedFeaturesNum++;
            }
        }


        // TODO: ...



    }

    public void removeLostFeatures() {
        // Remove the features that lost track.
        int jacobianRowSize = 0;
        List<Integer> invalidFeatureIds = new ArrayList<>();
        List<Integer> processedFeatureIds = new ArrayList<>();

        for (Feature feature : mapServer.values()) {
            // Pass the features that are still being tracked.
            if (feature.observations.containsKey(stateServer.imuState.id)) {
                continue;
            }
            if (feature.observations.size() < 3) {
                invalidFeatureIds.add(feature.id);
                continue;
            }

            // Check if the feature can be initialized if it
            // has not been.
            if (!feature.isInitialized()) {
                if (!feature.checkMotion(stateServer.camStates)) {
                    invalidFeatureIds.add(feature.id);
                    continue;
                } else if (!feature.initializePosition(stateServer.camStates)) {
                    invalidFeatureIds.add(feature.id);
                    continue;
                }
            }//if


            jacobianRowSize += 4 * feature.observations.size() - 3;
            processedFeatureIds.add(feature.id);
        }//for


        // Remove the features that do not have enough measurements.
        for (final Integer featureId : invalidFeatureIds) {
            mapServer.remove(featureId);
        }


        // Return if there is no lost feature to be processed.
        if (processedFeatureIds.isEmpty()) return;

        Mat Hx = Mat.zeros(jacobianRowSize, 21 + 6 * stateServer.camStates.size(), CV_64F); // TODO: is .zeroes the Java equivalent to c++ .Zero?
        Mat r = Mat.zeros(jacobianRowSize, 1, CV_64F);
        int stackCntr = 0;

        // Process the features which lose track.
        for (final Integer featureId : processedFeatureIds) {
            Feature feature = mapServer.get(featureId);

            List<Integer> camStateIds = new ArrayList<>(feature.observations.keySet());

            Mat Hxj = new Mat();
            Mat rj = new MatOfDouble();
            featureJacobian(feature.id, camStateIds, Hxj, rj);

            if (gatingTest(Hxj, rj, camStateIds.size()-1)) {
                Hxj.copyTo(Hx.submat()); // TODO: understand Block method
                r.segment(); // TODO: segment?
                stackCntr += Hxj.rows();
            }

            // Put an upper bound on the row size of measurement Jacobian,
            // which helps guarantee the executation time.
            if (stackCntr > 1500) break;
        }

        Hx.conservativeResize(); // TODO
        r.conservativeResize(); // TODO

        // Perform the measurement update step.
        measurementUpdate(Hx, r);

        // Remove all processed features from the map.
        for (final Integer featureId : processedFeatureIds) {
            mapServer.remove(featureId);
        }


    }

    public void measurementUpdate(final Mat H, final MatOfDouble r) {
        if (H.rows() == 0 || r.rows() == 0) return;

        // Decompose the final Jacobian matrix to reduce computational
        // complexity as in Equation (28), (29).
        Mat Hthin;
        MatOfDouble rThin;

        if (H.rows() > H.cols()) {
            // Convert H to a sparse matrix.
            H.sp; // TODO: BIG PROBLEM! --> See Obsidian
        }
    }

    public void gatingTest() {}

    public void featureJacobian(final Integer featureId, final List<Integer> camStateIds, RealMatrix Hx, RealVector r) {
        final Feature feature = mapServer.get(featureId);

        // Check how many camera states in the provided camera
        // id camera has actually seen this feature.
        List<Integer> validCamStateIds = new ArrayList<>();
        for (final Integer camId : camStateIds ) {
            if (feature.observations.containsKey(camId)) {
                validCamStateIds.add(camId);
            }
        }

        int jacobianRowSize = 2 * validCamStateIds.size(); // 4 * size()  in stereo

        RealMatrix Hxj = MatrixUtils.createRealMatrix(jacobianRowSize, 15+stateServer.camStates.size()*6); // 21 + ... * 6 in stereo
        RealMatrix Hfj = MatrixUtils.createRealMatrix(jacobianRowSize, 3);
        RealVector rj = new ArrayRealVector(jacobianRowSize);
        int stackCntr = 0;

        for (final Integer camId : validCamStateIds) {
            Mat Hxi = Mat.zeros(4,6,CV_64F);
            Mat Hfi = Mat.zeros(4,3,CV_64F);
            MatOfDouble ri = Vec2d.create(); // vec2 instead of vec4, bc. mono
            measurementJacobian(camId, feature.id, Hxi, Hfi, ri);

            // TODO: cameStateIter
            int camStateCntr; // TODO: initialize

            // Stack the Jacobians.
            // TODO: Is overriding Hxi, Hfi (input parameters) correct?
            Hxi.copyTo(Hxj.submat(new Rect(21+6*camStateCntr,stackCntr,6,4)));
            Hfi.copyTo(Hfj.submat(new Rect(0,stackCntr,3,4)));
            rj.segment(); // TODO
            stackCntr += 4;
        }

        // Project the residual and Jacobians onto the nullspace
        // of H_fj. // TODO
        Mat _w = new Mat();
        Mat u = new Mat();
        Mat _vt = new Mat();
        Core.SVDecomp(Hfj, _w, u, _vt); // TODO: is the u output actually equivalent to the Python u? // TODO: which flags?
        Mat A = u.submat(); // TODO

        A.t().matMul(Hxj).copyTo(Hx);
        A.t().matMul(rj).copyTo(r);


    }
    public void measurementJacobian(final Integer camStateId, final Integer featureId, RealMatrix Hx, RealMatrix Hf, RealVector r) {
        // Prepare all the required data.
        final CamState camState = stateServer.camStates.get(camStateId);
        final Feature feature = mapServer.get(featureId);

        // Cam0 pose.
        RealMatrix Rwc0 = quaternionToRotation(camState.orientation);
        final RealVector tc0w = camState.position;

        // 3d feature position in the world frame.
        // And its observation with the stereo cameras.
        final RealVector pw = feature.position;
        final RealVector z = feature.observations.get(camStateId);

        // Convert the feature position from the world frame to
        // the cam0 and cam1 frame.
        RealVector pc0 = Rwc0.operate(pw.subtract(tc0w));

        // Compute the Jacobians.
        double X = pc0.getEntry(0);
        double Y = pc0.getEntry(1);
        double Z = pc0.getEntry(2);

        RealMatrix Ji = MatrixUtils.createRealMatrix(2,3);
        Ji.setRow(0, new double[]{1,0,-X / Z});
        Ji.setRow(1, new double[]{0,1,-Y / Z});
        Ji = Ji.scalarMultiply(1.0d/Z);

        // Enforce observability constraint
        RealMatrix A = MatrixUtils.createRealMatrix(2,6);
        A.setSubMatrix(Ji.multiply(skewSymmetric(pc0)).getData(), 0,0); // TODO
        A.setSubMatrix(Ji.scalarMultiply(-1).multiply(quaternionToRotation(camState.)),0,3); // TODO: camState.Orientation or OrientationNull?
        RealVector u = new ArrayRealVector(6);

        u.setSubVector(0, quaternionToRotation(camState.orientationNull).operate(ImuState.gravity)); // TODO: is orientNULL correct?
        u.setSubVector(3, skewSymmetric(pw.subtract(camState.positionNull)).operate(ImuState.gravity)); // TODO: is posNULL correct?

        Hx.setSubMatrix(A.subtract(A.operate(u).mapDivide(u.dotProduct(u)).outerProduct(u)).getData(), 0,0);
        assert(Hx.getRowDimension() == 2 && Hx.getColumnDimension() == 6);
        Hf.setSubMatrix(Hx.getSubMatrix(0,1,3,5).scalarMultiply(-1).getData(),0,0);
        assert(Hf.getRowDimension() == 2 && Hf.getColumnDimension() == 3);

        // TODO: Why in C++ does H_f get calculated and then COMPLETELY overridden?

        // Compute the residual.
        r.setSubVector(0, MatrixUtils.createRealVector(new double[]{X/Z, Y/Z}));
    }
    // TODO: are there any other methods where you are supposed to edit the input params, but I accidentally just reassigned the param?


    public void pruneCamStateBuffer() {}

    public void publish() {}

    public void onlineReset() {}


}
