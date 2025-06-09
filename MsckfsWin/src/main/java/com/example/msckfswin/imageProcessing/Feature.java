package com.example.msckfswin.imageProcessing;

import static com.example.msckfswin.utils.MathUtils.quaternionToRotation;
import static org.apache.commons.math3.linear.MatrixUtils.createRealVector;

import com.example.msckfswin.CamState;
import com.example.msckfswin.utils.Isometry3d;
import com.example.msckfswin.utils.MathUtils.*;
import com.example.msckfswin.utils.Vec3d;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealVector;
import org.opencv.core.MatOfDouble;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class Feature {

    public final int id;

    public final Map<Integer, RealVector> observations; // <int, vec2d>. Originally vec4d.

    // 3d postion of the feature in the world frame.
    public RealVector position = Vec3d.create();
    public Feature(int id,) {
        this.id = id;
        this.observations = new HashMap<>();
    }



    public boolean checkMotion(final Map<Integer, CamState> camStates) {

        final Integer firstCamId = Collections.min(): // TODO: is min and max needed? or smth else?
        final Integer lastCamId;

        Isometry3d firstCamPose = new Isometry3d(quaternionToRotation(camStates.get(firstCamId).orientation).t(), camStates.get(firstCamId).position); // TODO: transpose or nah?
        Isometry3d lastCamPose; // TODO

        // Get the direction of the feature when it is first observed.
        // This direction is represented in the world frame.
        RealVector featureDirection = createRealVector(new double[]{,,1.0}); // TODO
        featureDirection = featureDirection.mapMultiply( 1 / featureDirection.getNorm());
        featureDirection = firstCamPose.R.preMultiply(featureDirection); // TODO: preMultiply or operate()? https://stackoverflow.com/questions/34754071/how-do-i-postmultiply-a-realmatrix-by-a-realvector-org-apache-commons-commons-m

        // Compute the translation between the first frame
        // and the last frame. We assume the first frame and
        // the last frame will provide the largest motion to
        // speed up the checking process.
        RealVector translation = lastCamPose.t.subtract(firstCamPose.t);
        double parallel = translation.dotProduct(featureDirection);
        RealVector orthogonalTranslation = translation.subtract(featureDirection.mapMultiply(parallel));

        return orthogonalTranslation.getNorm() > OptimizationConfig.translation_threshold;
    }

    public static class OptimizationConfig {

        // TODO: Differs between Python and C++

        public static double translation_threshold = 0.2;
        public static double huber_epsilon = 0.01;
        public static double estimation_precision = 5e-7;
        public static double initial_damping = 1e-3;
        public static int outer_loop_max_iteration = 10;
        public static int innerLoopMaxIteration = 10;
    }


}
