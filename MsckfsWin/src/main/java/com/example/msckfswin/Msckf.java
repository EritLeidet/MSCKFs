package com.example.msckfswin;

import static com.example.msckfswin.utils.MatOperations.add;
import static com.example.msckfswin.utils.MatOperations.div;
import static com.example.msckfswin.utils.MatOperations.mult;
import static com.example.msckfswin.utils.MatOperations.neg;
import static com.example.msckfswin.utils.MatOperations.sub;
import static com.example.msckfswin.utils.MathUtils.*;
import static com.example.msckfswin.utils.Matx.setD;

import static org.apache.commons.math3.linear.MatrixUtils.createRealIdentityMatrix;
import static org.apache.commons.math3.linear.MatrixUtils.createRealMatrix;
import static org.opencv.core.CvType.CV_64F;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.example.msckfswin.config.ProcessorConfig;
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


import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.CholeskyDecomposition;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.QRDecomposition;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.linear.SingularValueDecomposition;
import org.apache.commons.numbers.quaternion.Quaternion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

public class Msckf {

    // State vector
    private StateServer stateServer;

    private ProcessorConfig config;

    // Features used
    private Map<Integer, Feature> mapServer;

    // IMU data buffer
    // This is buffer is used to handle the unsynchronization or
    // transfer delay between IMU and Image messages.
    private final List<ImuMessage> imuMsgBuffer = Collections.synchronizedList(new ArrayList<>()); // TODO: should be same as in ImageProcessor

    // Tracking rate.
    private double trackingRate;

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
        RealMatrix F = createRealMatrix(21,21); // TODO: are these correct sizes for mono?
        RealMatrix G = createRealMatrix(21,12);

        // F.block
        F.setSubMatrix(skewSymmetric(gyro).scalarMultiply(-1).getData(), 0,0);
        F.setSubMatrix(createRealIdentityMatrix(3).scalarMultiply(-1).getData(),0,3);
        F.setSubMatrix(quaternionToRotation(imuState.orientation).transpose().multiply(skewSymmetric(acc)).scalarMultiply(-1).getData(),6,0);
        F.setSubMatrix(quaternionToRotation(imuState.orientation).transpose().scalarMultiply(-1).getData(),6,9);
        F.setSubMatrix(createRealIdentityMatrix(3).getData(),12,6);

        // G.block
        G.setSubMatrix(createRealIdentityMatrix(3).scalarMultiply(-1).getData(),0,0);
        G.setSubMatrix(createRealIdentityMatrix(3).getData(),3,3);
        G.setSubMatrix(quaternionToRotation(imuState.orientation).transpose().scalarMultiply(-1).getData(),6,6);
        G.setSubMatrix(createRealIdentityMatrix(3).getData(),9,9);

        // Approximate matrix exponential to the 3rd order,
        // which can be considered to be accurate enough assuming
        // dtime is within 0.01s.
        RealMatrix Fdt = F.scalarMultiply(dtime);
        RealMatrix FdtSquare = Fdt.multiply(Fdt);
        RealMatrix FdtCube = FdtSquare.multiply(Fdt);
        // TODO: danger! 21 -> mono?
        RealMatrix Phi = createRealIdentityMatrix(21).add(Fdt).add(FdtSquare.scalarMultiply(0.5)).add(FdtCube.scalarMultiply(1.0/6.0)).

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
        assert(Q.isSquare() && Q.getRowDimension() == 21);
        stateServer.stateCov.setSubMatrix(Phi.multiply(stateServer.stateCov).getSubMatrix(0,20,0,20).multiply(Phi.transpose()).add(Q).getData(),0,0);

        if (!stateServer.camStates.isEmpty()) {
            // TODO: danger! 21 -> mono?
            RealMatrix colsSubmat = stateServer.stateCov.getSubMatrix(0,20,21,); // TODO: 20 correct?
            RealMatrix rowsSubmat = stateServer.stateCov.getSubMatrix(21,,0,);
            stateServer.stateCov.setSubMatrix(Phi.multiply(colsSubmat).getData(),,);
            stateServer.stateCov.setSubMatrix(rowsSubmat.multiply(Phi.transpose()).getData(),,);
        }

        RealMatrix stateCovFixed = (stateServer.stateCov.add(stateServer.stateCov.transpose())).scalarMultiply(0.5);
        stateServer.stateCov = stateCovFixed;

        // Update the state correspondes to null space.
        imuState.orientationNull = imuState.orientation;
        imuState.positionNull = imuState.position;
        imuState.velocityNull = imuState.velocity;

        // Update the state info
        stateServer.imuState.timestamp = timestamp;

    }

    public void predictNewState(final double dt, final RealVector gyro, final RealVector acc) {

        double gyroNorm = gyro.getNorm();

        RealMatrix Omega = createRealMatrix(4,4);
        Omega.setSubMatrix(skewSymmetric(gyro).scalarMultiply(-1).getData(),0,0);
        insertColumnVector(Omega,gyro,0,3);
        insertRowVector(Omega, gyro.mapMultiply(-1),3,0);

        Quaternion q = stateServer.imuState.orientation;
        RealVector v = stateServer.imuState.velocity;
        RealVector p = stateServer.imuState.position;

        // Some pre-calculation
        Quaternion dqDt, dqDt2;
        if (gyroNorm > 1e-5) {
            RealVector qVec = quaternionToVector(q);
            dqDt = vectorToQuaternion(
                    (createRealIdentityMatrix(4).scalarMultiply(cos(gyroNorm*dt*0.5))
                            .add(Omega.scalarMultiply(1/gyroNorm*sin(gyroNorm*dt*0.5))) // addition before multiplication
                            .operate(qVec)
                    )
            );

            dqDt2 = vectorToQuaternion(
                    (createRealIdentityMatrix(4).scalarMultiply(cos(gyroNorm*dt*0.25))
                            .add(Omega.scalarMultiply(1/gyroNorm*sin(gyroNorm*dt*0.25)))
                            .operate(qVec)
                    )
            );


        } else {
            RealVector qVec = quaternionToVector(q);
            dqDt = vectorToQuaternion(
                    createRealIdentityMatrix(4).add(Omega.scalarMultiply(0.5*dt))
                            .scalarMultiply(cos(gyroNorm*dt*0.5))
                            .operate(qVec)
            );

            dqDt2 = vectorToQuaternion(
                    createRealIdentityMatrix(4).add(Omega.scalarMultiply(0.25*dt))
                            .scalarMultiply(cos(gyroNorm*dt*0.25))
                            .operate(qVec)
            );
        }


        RealMatrix dRDtTranspose = quaternionToRotation(dqDt).transpose();
        RealMatrix dRDt2Transpose = quaternionToRotation(dqDt2).transpose();

        // k1 = f(tn, yn)
        RealVector k1vDot = quaternionToRotation(q).transpose().operate(acc).add(ImuState.gravity);
        RealVector k1pDot = v; // TODO: create as copy or reference?

        // k2 = f(tn+dt/2, yn+k1*dt/2)
        RealVector k1v = v.add(k1vDot.mapMultiply(dt/2));
        RealVector k2vDot = dRDt2Transpose.operate(acc).add(ImuState.gravity);
        RealVector k2pDot = k1v;

        // k3 = f(tn+dt/2, yn+k2*dt/2)
        RealVector k2v = v.add(k2vDot).mapMultiply(dt/2);
        RealVector k3vDot = dRDt2Transpose.operate(acc).add(ImuState.gravity);
        RealVector k3pDot = k2v;

        // k4 = f(tn+dt, yn+k3*dt)
        RealVector k3v = v.add(k3vDot.mapMultiply(dt));
        RealVector k4vDot = dRDtTranspose.operate(acc).add(ImuState.gravity);
        RealVector k4pDot = k3v;

        // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)#
        q = dqDt.normalize();

        // update the imu state
        v = v.add(k1vDot.add(k2vDot.mapMultiply(2)).add(k3vDot.mapMultiply(2)).add(k4vDot).mapMultiply(dt/6));
        p = p.add(k1pDot.add(k2pDot.mapMultiply(2)).add(k3pDot.mapMultiply(2)).add(k4pDot).mapMultiply(dt/6));

        stateServer.imuState.orientation = q;
        stateServer.imuState.position = v;
        stateServer.imuState.velocity = p;

    }

    public void stateAugmentation(final long timestamp) { // TODO: OG datatype is double, but timestamps are long in java. Which better?
        final RealMatrix Ric = stateServer.imuState.rImuCam0;
        final RealVector tci = stateServer.imuState.tCam0Imu;

        // Add a new camera state to the state server.
        RealMatrix Rwi = quaternionToRotation(stateServer.imuState.orientation);
        RealMatrix Rwc = Ric.multiply(Rwi);
        RealVector tcw = stateServer.imuState.position.add(Rwi.transpose().operate(tci));
        CamState camState = new CamState(stateServer.imuState.id);
        stateServer.camStates.put(stateServer.imuState.id, camState);

        camState.timestamp = timestamp;
        camState.orientation = rotationToQuaternion(Rwc);
        camState.position = tcw;

        camState.orientationNull = camState.orientation;
        camState.positionNull = camState.position;

        // Update the covariance matrix of the state.
        // To simplify computation, the matrix J below is the nontrivial block
        // in Equation (16) in "A Multi-State Constraint Kalman Filter for Vision
        // -aided Inertial Navigation".

        RealMatrix J = createRealMatrix(6,21); // TODO: are these dimensions correct? Or should it be e.g. 15 instead of 21?
        J.setSubMatrix(Ric.getData(), 0,0);
        J.setSubMatrix(createRealIdentityMatrix(3).getData(),0,15);
        J.setSubMatrix(skewSymmetric(Rwi.transpose().operate(tci)).getData(),3,0);

        J.setSubMatrix(createRealIdentityMatrix(3).getData(),3,12);
        J.setSubMatrix(Rwi.transpose().getData(),3,18);

        // Resize the state covariance matrix.
        int oldRows = stateServer.stateCov.getRowDimension();
        RealMatrix stateCov = createRealMatrix(oldRows+6,oldRows+6); // TODO: should it really be +6, and not +3 or anything? What does the 6 stand for?
        stateServer.stateCov = ; // TODO: put at the end, like in Python

        // TODO: this is so weird. Python resizes stateCov, only then tries to get a copy of the original stateCov?
        RealMatrix P11 = stateServer.stateCov.getSubMatrix(0,oldRows-1,0,oldRows-1); // TODO: :21 like in Python or 15? Originally hard coded.

        // Fill in the augmented state covariance.
        RealMatrix r1 = J.multiply(P11); // Shape (6,15)
        stateCov.setSubMatrix(r1.getData(),oldRows,0); // TODO: oldRows or oldRows-1? Bc. Python uses old_rows
        stateCov.setSubMatrix(r1.transpose().getData(),oldRows,0); // TODO: why in Python not use r1?
        stateCov.setSubMatrix(J.multiply(P11).multiply(J.transpose()).getData(),oldRows,oldRows);

        // Fix the covariance to be symmetric
        stateServer.stateCov = stateCov.add(stateCov.transpose()).scalarMultiply(0.5);


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

        trackingRate = trackedFeaturesNum / (double) currFeatureNum;


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

        RealMatrix Hx = createRealMatrix(jacobianRowSize, 15+6*stateServer.camStates.size()); // 21 + 6 * size() in stereo
        RealVector r = new ArrayRealVector(jacobianRowSize);
        int stackCntr = 0;

        // Process the features which lose track.
        for (final Integer featureId : processedFeatureIds) {
            Feature feature = mapServer.get(featureId);

            List<Integer> camStateIds = new ArrayList<>(feature.observations.keySet());


            ImmutablePair<RealMatrix,RealVector> HxjRj = featureJacobian(feature.id, camStateIds);
            RealMatrix Hxj = HxjRj.left; // TODO: maybe call Tuple instead of resundant vars?
            RealVector rj = HxjRj.right;

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

    public void gatingTest(final RealMatrix H, final RealVector r, final int dof) {
        RealMatrix P1 = H.multiply(stateServer.stateCov).multiply(H.transpose());
        assert(P1.isSquare());
        RealMatrix P2 = createRealIdentityMatrix(H.getRowDimension()).scalarMultiply(config.observationNoise);
        double gamma = new DecompositionSolver().g; // TODO: r.transpose()
        // TODO
    }

    public ImmutablePair<RealMatrix, RealVector> featureJacobian(final Integer featureId, final List<Integer> camStateIds) {
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

        RealMatrix Hxj = createRealMatrix(jacobianRowSize, 15+stateServer.camStates.size()*6); // 21 + size() * 6 in stereo
        RealMatrix Hfj = createRealMatrix(jacobianRowSize, 3);
        RealVector rj = new ArrayRealVector(jacobianRowSize);
        int stackCntr = 0;

        for (final Integer camId : validCamStateIds) {
            RealMatrix Hxi = createRealMatrix(2,6); // (4,6) in stereo
            RealMatrix Hfi = createRealMatrix(2,3); // (4,3) in stereo
            RealVector ri = new ArrayRealVector(2); // size 4 in stereo
            measurementJacobian(camId, feature.id, Hxi, Hfi, ri);

            int camStateCntr = new ArrayList<>(stateServer.camStates.keySet()).indexOf(camId); // TODO: do the keys() have to be sorted, for .indexOf() to be consistent

            // Stack the Jacobians. // TODO: why are the indices so different in Python impl? Why starts at +4 there?
            Hxj.setSubMatrix(Hxi.getData(), stackCntr ,15+6*camStateCntr); // column 21+6*camStateCntr in stereo
            Hfj.setSubMatrix(Hfi.getData(),stackCntr,0);
            rj.setSubVector(stackCntr,ri);
            stackCntr += 2; // += 4 in stereo
        }

        // Project the residual and Jacobians onto the nullspace
        // of H_fj.
        RealMatrix U = new SingularValueDecomposition(Hfj).getU();
        RealMatrix A = U.getSubMatrix(0,U.getRowDimension()-1,3,U.getColumnDimension()-1);

        Hx = A.transpose().multiply(Hxj);
        r = A.transpose().operate(rj);
        return new ImmutablePair<>(Hx,r);
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

        RealMatrix Ji = createRealMatrix(2,3);
        Ji.setRow(0, new double[]{1,0,-X / Z});
        Ji.setRow(1, new double[]{0,1,-Y / Z});
        Ji = Ji.scalarMultiply(1.0d/Z);

        // Enforce observability constraint
        RealMatrix A = createRealMatrix(2,6);
        A.setSubMatrix(Ji.multiply(skewSymmetric(pc0)).getData(), 0,0); // TODO
        A.setSubMatrix(Ji.scalarMultiply(-1).multiply(quaternionToRotation(camState.orientationNull)).getData(),0,3); // TODO: camState.Orientation or OrientationNull?
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
