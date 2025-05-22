package com.example.msckfs.imageProcessing;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.BORDER_REFLECT_101;
import java.util.concurrent.Flow.Publisher;
import com.example.msckfs.FeatureIDtype;
import com.example.msckfs.imuProcessing.ImuMessage;
import com.example.msckfs.utils.MathUtils;
import com.example.msckfs.utils.MathUtils.*;
import com.example.msckfs.utils.Matx33f;
import com.example.msckfs.utils.Vec3d;
import com.example.msckfs.utils.Vec3f;
import com.example.msckfs.utils.Vec4d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.video.Video;
import org.opencv.features2d.FastFeatureDetector;
import org.opencv.calib3d.Calib3d.*;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.Flow;
import java.util.concurrent.SubmissionPublisher;

public class ImageProcessor {

    //TODO: add "final" to more things"


    private final ProcessorConfig processorConfig;

    // Indicate if this is the first image message.
    private boolean isFirstImg;

    // ID for the next new feature.
    private int next_feature_id;

    // Feature detector
    private FastFeatureDetector featureDetector;

    // IMU message buffer
    // TODO: does msgBuffer have to be static, or pass reference to methods?
    // TODO Is static gonna cascade into making everything static? Would that be ok / make sense? How do you usually access fields in java asynchronously?
    // TODO: do you initialize with null + instantiate() method or right here?
    private static final List<ImuMessage> imuMsgBuffer = Collections.synchronizedList(new LinkedList<>()); // TODO: Collections.synchronizedList(new ArrayList<>()); in constructor, non-static? Possible?


    // Previous and current images //TODO: data types
    private ImageMessage cam0PrevImgMsg;
    private ImageMessage cam0CurrImgMsg;

    // Pyramids for previous and current image
    private List<Mat> prevCam0Pyramid;
    private List<Mat> currCam0Pyramid;

    // Features in the previous and current image.
    private GridFeatures prevFeatures;
    private GridFeatures currFeatures;

    private Mat TCam0Imu;
    private Mat RCam0Imu;
    private Mat tCam0Imu;

    private final SubmissionPublisher<FeatureMessage> featurePublisher;
    public ImageProcessor(ProcessorConfig processorConfig, SubmissionPublisher<FeatureMessage> featurePublisher) {
        this.processorConfig = processorConfig;
        this.isFirstImg = true;
        this.next_feature_id = 0;
        this.featureDetector = FastFeatureDetector.create(processorConfig.fast_threshold) // TODO: threshold
        this.imuMsgBuffer = new ArrayBlockingQueue(); //TODO: maybe LinkedBlockingQueue? In Python just '[]'. Wait til unterstand code better.
            //TODO: imuMsgBuffer Capacity
        this.cam0_curr_img_msg = null;
        this.cam0_prev_img_msg = null;
        this.featurePublisher = featurePublisher;
    }



    public boolean loadParameters() {
        //TODO
    }

    public boolean initialize() {
        //TODO
    }


    public void imageCallback(final Mat cam0_img) {
        double startTime;

        // Get the current image.
        Mat cam0_curr_img = cam0_img; //TODO: change to Mono8 encoding? or is it FROM Mono8 encoding?

        createImagePyramids();


        // Detect features in the first frame.
        if (isFirstImg) {
            startTime = Instant.now().getEpochSecond();

            //TODO: implement ROS_INFO logging equivalent
            initializeFirstFrame();
            //ROS_INFO("Detection time: %f",
            //    (ros::Time::now()-start_time).toSec());
            isFirstImg = false;

            // Draw results
            startTime = Instant.now().getEpochSecond();
            drawFeatures();
            //ROS_INFO("Draw features: %f",
            //    (ros::Time::now()-start_time).toSec());
        } else {
            // Track the feature in the previous image.
            startTime = Instant.now().getEpochSecond();
            addNewFeatures();
            //ROS_INFO("Addition time: %f",
            //    (ros::Time::now()-start_time).toSec());

            // Add new features into the current image.
            startTime = Instant.now().getEpochSecond();
            pruneGridFeatures();
            //ROS_INFO("Prune grid features: %f",
            //    (ros::Time::now()-start_time).toSec());

            // Draw results
            startTime = Instant.now().getEpochSecond();
            drawFeatures();
            //ROS_INFO("Draw features: %f",
            //    (ros::Time::now()-start_time).toSec());

        }


        //ros::Time start_time = ros::Time::now();
        //updateFeatureLifetime();
        //ROS_INFO("Statistics: %f",
        //    (ros::Time::now()-start_time).toSec());

        // Publish features in the current image.
        startTime = Instant.now().getEpochSecond();
        publish();
        //ROS_INFO("Publishing: %f",
        //    (ros::Time::now()-start_time).toSec());

        // Update the previous image and previous features
        cam0PrevImgMsg = cam0CurrImgMsg;
        prevFeatures = currFeatures;
        List<Mat> tempPrev = prevCam0Pyramid;
        prevCam0Pyramid = currCam0Pyramid;
        currCam0Pyramid = tempPrev;

        // Initialize the current features to empty vectors. // TODO: decide on GridFeaturesDatatype and impl.
        currFeatures = new GridFeatures();
        for (int code = 0; code < processorConfig.grid_row*processorConfig.grid_col; code++) {
            currFeatures.put(code, new ArrayList<>());
        }
    }

    public void predictFeatureTracking(final MatOfPoint2f inputPts, final Matx33f Rpc, final Vec4d intrinsics, MatOfPoint2f compensatedPts) {
        List<Point> ptsList = inputPts.toList();
        List<Point> compPtsList = compensatedPts.toList();
        // Return directly if there are no input features.
        if (ptsList.isEmpty()) {
            compPtsList.clear();
            return;
        }
        compPtsList = compPtsList.subList(0, ptsList.size());

        // Intrinsic matrix.
        Matx33f K = new Matx33f(new float[] { // TODO: why the conversion?
                (float) intrinsics.get(0), 0.0f, (float) intrinsics.get(2),
                0.0f, (float) intrinsics.get(1), (float) intrinsics.get(3),
                0.0f, 0.0f, 1.0f});
        Matx33f H = (Matx33f) K.matMul(Rpc).matMul(K.inv());

        for (int i = 0; i < ptsList.size(); i++) {
            Vec3d p1 = new Vec3d(ptsList.get(i).x, ptsList.get(i).y, 1.0d); // Vec3d instead of Vec3f
            Vec3d p2 = (Vec3d) H.matMul(p1);
            compPtsList.get(i).x = p2.get(0) / p2.get(2);
            compPtsList.get(i).y = p2.get(1) / p2.get(2);

        }

        compensatedPts.fromList(compPtsList);

    }


    //TODO: imu msg datatype?
    public void imuCallback(Object ) {
        //TODO
    }

    public void createImagePyramids() {
        //TODO
        Mat curr_cam0_img = cam0_curr_img_msg.image; // TODO: img_msg format / data type?
        Video.buildOpticalFlowPyramid(curr_cam0_img, currCam0Pyramid, new Size(processorConfig.patch_size, processorConfig.patch_size), processorConfig.pyramid_levels, true, BORDER_REFLECT_101, BORDER_CONSTANT, false);
    }

    public void initializeFirstFrame() {
        //TODO
    }

    public void drawFeatures() {

    }

    public void addNewFeatures() {

    }

    public void pruneGridFeatures() {

    }

    public void publish() {

        // Publish features.
        // TODO:  CameraMeasurement object? not in Python?

        List<Integer> currIds = new ArrayList<>();
        List<Point> currCam0Points = new ArrayList<>();

        for (Map.Entry<Integer, List<FeatureMetaData>> gridFeatures : currFeatures.entrySet()) {
            for (FeatureMetaData feature : gridFeatures.getValue()) {
                currIds.add(feature.id);
                currCam0Points.add(feature.cam0_point);
            }
        }

        List<Point> currCam0PointsUndistorted = new ArrayList<>();

        undistortPoints(); // TODO

        List<FeatureMeasurement> features = new ArrayList<>();
        for (int i = 0; i < currIds.size(); i++) {
            features.add(new FeatureMeasurement(currIds.get(i), currCam0PointsUndistorted.get(i).x, currCam0PointsUndistorted.get(i).y));
        }


        // TODO: how to publish?
        FeatureMessage featureMessage = new FeatureMessage(getEp)
        featurePublisher.submit(features);

        // Publish tracking info. // TODO? For Debug?


    }

    public void undistortPoints() {
        // TODO
    }

    public void trackFeatures() {

        // TODO: " static int grid_height" <- Why "static"?
        // Size of each grid //TODO: which way to calculate it? Like in C++ or in Python?
        int grid_height = cam0CurrImg.rows() / processorConfig.grid_row;
        int grid_width = cam0CurrImg.cols() / processorConfig.grid_col;

        // Compute a rough relative rotation which takes a vector
        // from the previous frame to the current frame.
        Matx33f cam0Rpc;
        integrateImuData(cam0Rpc);

        // Organize the features in the previous image.
        List<Integer> prevIds = new ArrayList<>();
        List<Integer> prevLifetime = new ArrayList<>();
        MatOfPoint2f prevCam0Points = new MatOfPoint2f(); // TODO: put anything in the constructor?

        prevCam0Points.put(0,0, new Point());
        for (FeatureMetaData feature : prevFeatures) {
            // TODO: what exactly should be iterated over?
            // TODO: iterate
            prevIds.add(feature.id);
            prevLifetime.add(feature.lifetime);
            prevCam0Points.add(feature.cam0_point);
        }

        // Number of the features before tracking.
        int before_tracking = prevCam0Points.size();

        // Abort tracking if there is no features in
        // the previous frame.
        if (prevIds.isEmpty()) return;

        // Track features using LK optical flow method.
        // TODO: convert to list and then later call .fromList on them
        MatOfPoint2f currCam0Points; // TODO: C++ code says it's in a vector! Has to be iterable, for loop. But clashes with calcOpticalFlowPyrLK?
        MatOfByte trackInliers = new MatOfByte(); // TODO: CHANGE OBJECT DATATYPE
                // TODO: do I need to pass smt in constructor? MatOfByte

        predictFeatureTracking(prevCam0Points, cam0Rpc, processorConfig.cam0Intrinsics, currCam0Points); // TODO

        Video.calcOpticalFlowPyrLK(prevCam0Pyramid, currCam0Pyramid, prevCam0Points, currCam0Points, trackInliers, new MatOfFloat(), new Size(processorConfig.patch_size, processorConfig.patch_size), processorConfig.pyramid_levels, new TermCriteria(TermCriteria.COUNT+TermCriteria.EPS, processorConfig.max_iteration, processorConfig.track_precision), Video.OPTFLOW_USE_INITIAL_FLOW);
        // TODO: it is true that buildPyr gives List<Mat>, but calcPyr takes Mat. Both in C++ and Java. How use?

        // TODO: won't there be issues with conversion from RealVector to Point2f?
        // TODO: Mat datatype has push_back function. Maybe use Mat instead of Array?
        // TODO: noArray. does it matter what passed?

        // Mark those tracked points out of the image region
        // as untracked.
        // TODO: would iterating over .asList work like iterating over row, col? 1. In general / 2. Specifically row, col instead of col, row?
        // TODO: THIS ITERATION IS WRONG!!!
        // TODO: find a way to iterate over Mat directly, without making assumptions of matrix structure?
        for (int i = 0; i < currCam0Points.size().width; ++i) { // TODO: size().height or width?
            if (trackInliersList.get(i) == 0) continue; // TODO: I think it's a vector saved as a matrix? How to access?

            //TODO: whyyy does ByteMat give back byte[] and not byte? Is it an array w/ one entry?
            // TODO: if I decide to iterate over map w/ .get(): do I have to start with col or row?
        }
        // TODO: Google how to ByteMat edit entry
        // TODO: maybe convert List->Mat every time opticalFlow is called? Or is that risky bc. objects may have to be overwritten? Nahhh. In C++ some are called "InputArrays" or "InputOutputArrays".
        trackInliers.at(Byte.class, 0,0).setV((byte) 1); // TODO does .at.set() work? Overrides, not just creates?
        for (int row = 0; row < trackInliers.rows(); ++row) {
            for (int col = 0; col < trackInliers.cols(); ++col) {
                 if (trackInliers.at(Byte.class, row, col).getV() == 0) continue;
                Point currPoint = currCam0Points.at(Point.class, row, col).getV();
                // TODO: what is getV2c etc? does setV do what I think it does?
                if (    currPoint.y < 0 ||
                        currPoint.y > cam0CurrImg.rows() - 1 ||
                        currPoint.x < 0 ||
                        currPoint.x > cam0CurrImg.cols() -1) {
                    trackInliers.at(Byte.class, row, col).setV((byte) 0);
                }
            }
        } //for

        // Collect the tracked points.
        List<Integer> prevTrackedIds = new ArrayList<>(); // TODO: technically "FeatureIDType" like in FeatureMetaData
        MatOfInt prevTrackedLifetime = new MatOfInt();
        MatOfPoint2f prevTrackedCam0Points = new MatOfPoint2f(); // TODO: this used to be a vector in C++, so wouldn't a list make more sense?
        MatOfPoint2f currTrackedCam0Points = new MatOfPoint2f(); // TODO: this used to be a vector in C++, so wouldn't a list make more sense?


        /* // TODO
        removeUnmarkedElements(prevIds, trackInliers, prevTrackedIds);
        removeUnmarkedElements(prevLifetime, trackInliers, prevTrackedLifetime);
        removeUnmarkedElements(prevCam0Points, trackInliers, prevTrackedCam0Points);
        removeUnmarkedElements(prevCam0Points, trackInliers, currTrackedCam0Points);

         */


        // Number of features left after tracking.
        afterTracking = currTrackedCam0Points.size();

        // Step 1: stereo matching. (skipped)
        MatOfPoint2f currMatchedCam0Points = currTrackedCam0Points; // TODO: replace CurrMatched with currTracked if correct.

        // Step 2: // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1. // TODO: what are temporal image pairs? Do I need to know?
        MatOfInt cam0RansacInliers; // TODO: vector<int>?
        //TODO: curr_tracked_cam0_points oder curr_cam0_points verwenden? Bei cam0 wird curr_ verwendet, aber bei cam1 curr_tracked??? Z. 542 Stereo - In Python auch so.
        // TODO: where get cam0Intrinsics? etc.

        // TODO: RANSAC
        // twoPointRansac(prevTrackedCam0Points, currMatchedCam0Points, cam0Rpc, processorConfig.cam0Intrinsics, processorConfig.cam0DistortionModel, processorConfig.cam0DistortionCoeffs, processorConfig.ransac_threshold, 0.99, cam0RansacInliers);

        // Number of features after ransac.
        int after_ransac = 0;

        for (int row = 0; row < cam0RansacInliers.rows(); ++row) {
            for (int col = 0; col < cam0RansacInliers.cols(); ++col) {
                if (cam0RansacInliers.at(Integer.class, row, col).getV() == 0) continue;
                // TODO: static_cast?
                int row = currMatchedCam0Points.get();
                        // TODO: ...


            }
        } //for
        currFeatures.put(). // TODO: add to map
        // TODO
    }

    // TODO: replace all the Vec3f etc. with Mat, in cases where Mat needed to be casted to Vec3f as a result?
    // TODO: Alternatively: instead of casting, create a .fromMat(Mat) method
    public void integrateImuData(Matx33f cam0Rpc) {
        // Find the start and the end limit within the imu msg buffer.

        int beginIter = 0;
        // TODO: would iterator be more efficient? (unimportant?)
        for (int i = 0; i < imuMsgBuffer.size(); i++) { // TODO: give credit for loop to Python
            if (imuMsgBuffer.get(i).timestamp >= cam0PrevImgMsg.timestamp - 0.01) {
                beginIter = i;
                break;
            }
        }

        int endIter = beginIter;
        for (int i = beginIter; i < imuMsgBuffer.size(); i++) { // TODO: give credit for loop to Python
            if (imuMsgBuffer.get(i).timestamp >= cam0CurrImgMsg.timestamp - 0.004) {
                endIter = i;
                break;
            }
        }

        // Compute the mean angular velocity in the IMU frame.
        Mat mean_ang_vel = new Vec3f();
        for (int i = beginIter; i < endIter; i++) {
            Core.add(mean_ang_vel, imuMsgBuffer.get(i).angularVelocity, mean_ang_vel); // Python
        }


        if (endIter-beginIter > 0) {
            Core.multiply(mean_ang_vel, new Scalar(1.0f / (endIter - beginIter)), mean_ang_vel);
        }

        // Transform the mean angular velocity from the IMU
        // frame to the cam0 frames.
        Vec3f cam0MeanAngVel = (Vec3f) RCam0Imu.t().matMul(mean_ang_vel);

        // Compute the relative rotation.
        double dtime = cam0CurrImgMsg.timestamp - cam0PrevImgMsg.timestamp;

        Mat scaledAngVel = new Mat();
        Core.multiply(cam0MeanAngVel, new Scalar(dtime), scaledAngVel); //
        Calib3d.Rodrigues(scaledAngVel, cam0Rpc);
        cam0Rpc = (Matx33f) cam0Rpc.t();



        // Delete the useless and used imu messages.
        imuMsgBuffer.subList(0, endIter).clear();

    }

    /** // TODO: JavaDoc
     * Remove the unmarked elements within a vector.
     * @Param
     *
     */
    public void removeUnmarkedElements() {
        // TODO: Not used in Python? What does this even do?
    }

    public void twoPointRansac() {

        // TODO: Python commented out the call of this method. Maybe runs without?

    }


    // TODO: add a "Destructor"? ~ImageProcessor()
}
