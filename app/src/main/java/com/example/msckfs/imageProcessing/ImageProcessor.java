package com.example.msckfs.imageProcessing;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.BORDER_REFLECT_101;
import static org.opencv.core.CvType.CV_8U;

import com.example.msckfs.imuProcessing.ImuMessage;
import com.example.msckfs.utils.Matx33d;
import com.example.msckfs.utils.Matx33f;
import com.example.msckfs.utils.Vec2d;
import com.example.msckfs.utils.Vec3d;
import com.example.msckfs.utils.Vec3f;
import com.example.msckfs.utils.Vec4d;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.video.Video;
import org.opencv.features2d.FastFeatureDetector;

import java.security.Key;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.SubmissionPublisher;
import java.util.stream.Collectors;
import java.util.stream.Stream;

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


    // Previous and current images
    private ImageMessage cam0PrevImgMsg;
    private ImageMessage cam0CurrImgMsg;

    // Pyramids for previous and current image
    private List<Mat> prevCam0Pyramid;
    private List<Mat> currCam0Pyramid;

    // Features in the previous and current image.
    private GridFeatures prevFeatures;
    private GridFeatures currFeatures;

    // (Python) load config
    // Camera calibration parameters
    private final Vec2d cam0Resolution;
    private final Vec4d cam0Intrinsics;
    private final String cam0DistortionModel;
    private final Vec4d cam0DistortionCoeffs;
    private Mat TCam0Imu;
    private Mat RCam0Imu;
    private Mat tCam0Imu;

    private final SubmissionPublisher<FeatureMessage> featurePublisher;
    public ImageProcessor(ProcessorConfig processorConfig, SubmissionPublisher<FeatureMessage> featurePublisher) {
        this.processorConfig = processorConfig;
        this.isFirstImg = true;
        this.next_feature_id = 0;
        this.featureDetector = FastFeatureDetector.create(processorConfig.fast_threshold) // TODO: threshold
        // this.imuMsgBuffer = new ArrayBlockingQueue(); //TODO: maybe LinkedBlockingQueue? In Python just '[]'. Wait til unterstand code better.
            //TODO: imuMsgBuffer Capacity
        this.cam0CurrImgMsg = null;
        this.cam0PrevImgMsg = null;
        this.featurePublisher = featurePublisher;

        cam0Resolution = processorConfig.cam0Resolution;
        cam0Intrinsics = processorConfig.cam0Intrinsics;
        cam0DistortionModel = processorConfig.cam0DistortionModel;
        cam0DistortionCoeffs = processorConfig.cam0DistortionCoeffs;

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
        currFeatures.init(processorConfig);
    }

    public void drawFeatures() {

    }

    public void pruneGridFeatures() {
        for (Map.Entry<Integer, List<FeatureMetaData>> item : currFeatures.entrySet()) {
            List<FeatureMetaData> gridFeatures = item.getValue();
            // Continue if the number of features in this grid does
            // not exceed the upper bound.
            if (gridFeatures.size() <= processorConfig.gridMaxFeatureNum) continue;
            item.getValue().sort((f0, f1) -> Integer.compare(f1.lifetime, f0.lifetime)); // descending order
            gridFeatures.subList(processorConfig.gridMaxFeatureNum,gridFeatures.size()).clear();
            assert(gridFeatures.size() == processorConfig.gridMaxFeatureNum); // TODO: remove after successful run

        }

    }

    public void addNewFeatures() {
        final Mat currImg = cam0CurrImgMsg.image;

        // Size of each grid.
        int gridHeight = currImg.rows() / processorConfig.gridRow;
        int gridWidth = currImg.cols() / processorConfig.gridCol;

        // Create a mask to avoid redetecting existing features.
        Mat mask = new Mat(currImg.rows(), currImg.cols(), CV_8U, new Scalar(1));

        for (Map.Entry<Integer, List<FeatureMetaData>> gridFeatures: currFeatures.entrySet()) {
            for (FeatureMetaData feature : gridFeatures.getValue()) {
                final int x = (int) feature.cam0_point.x;
                final int y = (int) feature.cam0_point.y;

                int upLim = y-2, bottomLim = y+3, leftLim = x-2, rightLim = x+3;
                if (upLim < 0) upLim = 0;
                if (bottomLim > currImg.rows()) bottomLim = currImg.rows();
                if (leftLim < 0) leftLim = 0;
                if (rightLim > currImg.cols()) rightLim = currImg.cols();

                Range rowRange = new Range(upLim, bottomLim);
                Range colRange = new Range(leftLim, rightLim);
                mask.submat(rowRange, colRange).setTo(new Scalar(0));
            }
        } // loop

        // Detect new features
        MatOfKeyPoint newFeatures = new MatOfKeyPoint();
        List<KeyPoint> newFeatList = newFeatures.toList(); // TODO: if newFeature-List is edited, the orginal MatOf needs to be updated!(fromList())
        featureDetector.detect(currImg, newFeatures, mask);

        // Collect the new detected features based on the grid.
        // Select the ones with top response within each grid afterwards
        List<List<KeyPoint>> newFeatureSieve = Stream.generate(() -> new ArrayList<KeyPoint>()).limit((long) processorConfig.gridRow * processorConfig.gridCol).collect(Collectors.toList()); // List of empty lists.

        for (KeyPoint feature: newFeatList) {
            int row = (int) feature.pt.y / gridHeight;
            int col = (int) feature.pt.x / gridWidth;
            newFeatureSieve.get(row * processorConfig.gridCol + col).add(feature);
        }

        newFeatList.clear(); // TODO: update MatOfKeyPoint accordingly!, later, after the list is last edited
        for (List<KeyPoint> item : newFeatureSieve) {
            if (item.size() > processorConfig.gridMaxFeatureNum) {
                item.sort((kp0, kp1) -> Float.compare(kp1.response, kp0.response) ); // descending order
                item.subList(processorConfig.gridMaxFeatureNum,item.size()).clear();

            }
            newFeatList.addAll(item);
        }

        int detectedNewFeatures = newFeatList.size();

        // Find the stereo matched points for the newly
        // detected features.
        // (skipped. cam0_points used instead of cam0_inliers from now on)

        List<Point> cam0Points = newFeatList.stream().map(keyPoint -> keyPoint.pt).collect(Collectors.toList());


        // Group the features into grids
        GridFeatures gridNewFeatures = groupFeatures(cam0Points, newFeatList, gridHeight, gridWidth);

        int newAddedFeatureNum = 0;
        // Collect new features within each grid with high response.
        // TODO: loop not yet accurate
        for (int code = 0; code < processorConfig.gridRow*processorConfig.gridCol; code++) {
            List<FeatureMetaData> featuresThisGrid = currFeatures.get(code);
            List<FeatureMetaData> newFeaturesThisGrid = gridNewFeatures.get(code);

            if (featuresThisGrid.size() >= processorConfig.gridMinFeatureNum) continue;

            int vacancyNum = processorConfig.gridMinFeatureNum - featuresThisGrid.size();
            for (int k = 0; k < vacancyNum && vacancyNum < newFeaturesThisGrid.size(); k++) {
                featuresThisGrid.add(newFeaturesThisGrid.get(k));
                featuresThisGrid.get(featuresThisGrid.size()-1).setId(next_feature_id++);
                featuresThisGrid.get(featuresThisGrid.size()-1).setLifetime(1);

                newAddedFeatureNum++;
            }
        }

    }

    /**
     * Group features into grid and sort features descending by response
     * @param cam0Points
     */
    private GridFeatures groupFeatures(List<Point> cam0Points, List<KeyPoint> newFeatList, int gridHeight, int gridWidth) {
        // Group the features into grids
        GridFeatures gridNewFeatures = new GridFeatures();
        gridNewFeatures.init(processorConfig);

        for (int i = 0; i < cam0Points.size(); i++) {
            final Point cam0Point = cam0Points.get(i);
            final float response = newFeatList.get(i).response;

            int row = (int) cam0Point.y / gridHeight;
            int col = (int) cam0Point.x / gridWidth;
            int code = row * processorConfig.gridCol + col;

            FeatureMetaData newFeature = new FeatureMetaData(response, cam0Point);
            gridNewFeatures.get(code).add(newFeature);
        }

        // Sort the new features in each grid based on its response.
        for (Map.Entry<Integer, List<FeatureMetaData>> item : gridNewFeatures.entrySet()) {
            item.getValue().sort((f0, f1) -> Float.compare(f1.response, f0.response)); // descending order
        }

        return gridNewFeatures;
    }






    public void initializeFirstFrame() {
        // Size of each grid.
        final Mat img = cam0CurrImgMsg.image;
        int gridHeight = img.rows() / processorConfig.gridRow;
        int gridWidth = img.cols() / processorConfig.gridCol;

        // Detect new features on the first image.
        MatOfKeyPoint newFeatures = new MatOfKeyPoint();
        featureDetector.detect(img, newFeatures);

        // Find the stereo matched points for the newly
        // detected features.
        // (skipped. cam0_points used instead of cam0_inliers from now on.)
        // TODO: info: code duplication. Make sure stays consistent
        List<KeyPoint> newFeatList = newFeatures.toList();
        List<Point> cam0Points = newFeatList.stream().map(keyPoint -> keyPoint.pt).collect(Collectors.toList());

        // (skipped)

        // Group the features into grids
        GridFeatures gridNewFeatures = groupFeatures(cam0Points, newFeatList, gridHeight, gridWidth);

        // Collect new features within each grid with high response.
        for (int code = 0; code < processorConfig.gridRow*processorConfig.gridCol; code++) {
            List<FeatureMetaData> featuresThisGrid = currFeatures.get(code);
            List<FeatureMetaData> newFeaturesThisGrid = gridNewFeatures.get(code);

            for (int k = 0; k < processorConfig.gridMinFeatureNum && k < newFeaturesThisGrid.size(); k++) {
                featuresThisGrid.add(newFeaturesThisGrid.get(k));
                featuresThisGrid.get(featuresThisGrid.size()-1).setId(next_feature_id++);
                featuresThisGrid.get(featuresThisGrid.size()-1).setLifetime(1);
            }
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
        Matx33d K = getK(intrinsics);
        Matx33d H = (Matx33d) K.matMul(Rpc).matMul(K.inv());

        for (int i = 0; i < ptsList.size(); i++) {
            Vec3d p1 = new Vec3d(ptsList.get(i).x, ptsList.get(i).y, 1.0d); // Vec3d instead of Vec3f
            Vec3d p2 = (Vec3d) H.matMul(p1);
            compPtsList.get(i).x = p2.get(0) / p2.get(2);
            compPtsList.get(i).y = p2.get(1) / p2.get(2);

        }

        compensatedPts.fromList(compPtsList);

    }

    private Mat getK(final Vec4d intrinsics) {
        return Matx33d.create(new double[] { // TODO: originally float[]. Maybe change back?
                intrinsics.get(0), 0.0f, intrinsics.get(2),
                0.0f, intrinsics.get(1), intrinsics.get(3),
                0.0f, 0.0f, 1.0f});
    }



    public void createImagePyramids() {
        final Mat curr_cam0_img = cam0CurrImgMsg.image;
        Video.buildOpticalFlowPyramid(curr_cam0_img, currCam0Pyramid, new Size(processorConfig.patch_size, processorConfig.patch_size), processorConfig.pyramid_levels, true, BORDER_REFLECT_101, BORDER_CONSTANT, false);
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

        undistortPoints(currCam0Points, cam0Intrinsics, cam0DistortionModel, cam0distortionCoeffs, currCam0PointsUndistorted); // TODO

        List<FeatureMeasurement> features = new ArrayList<>();
        for (int i = 0; i < currIds.size(); i++) {
            features.add(new FeatureMeasurement(currIds.get(i), currCam0PointsUndistorted.get(i).x, currCam0PointsUndistorted.get(i).y));
        }

        FeatureMessage featureMessage = new FeatureMessage(cam0CurrImgMsg.timestamp, features);
        featurePublisher.submit(featureMessage);

        // Publish tracking info. // TODO? For Debug?


    }

    public void undistortPoints(final MatOfPoint2f ptsIn, final Vec4d intrinsics, final String distortionModel, final Vec4d distortionCoeffs, final MatOfPoint2f ptsOut, final Matx33d rectificationMatrix, final Vec4d newIntrinsics) {

        if (ptsIn.total() == 0) return;

        final Matx33d K = getK(intrinsics);
        final Matx33d newK = getK(newIntrinsics);

        if (distortionModel.equals("equidistant")) {
            Calib3d.fisheye_undistortPoints(ptsIn, K.mat, distortionCoeffs, rectificationMatrix.mat, newK.mat);
        } else {
            Calib3d.undistortPoints(ptsIn, ptsOut, K.mat, distortionCoeffs, rectificationMatrix.mat, newK.mat);
        }

    }

    public void undistortPoints(final MatOfPoint2f ptsIn, final Vec4d intrinsics, final String distortionModel, final Vec4d distortionCoeffs, final MatOfPoint2f ptsOut) {
        undistortPoints(ptsIn, intrinsics, distortionModel, distortionCoeffs, ptsOut, Matx33d.eye(), new Vec4d(1,1,0,0));
    }

    public void trackFeatures() {

        // Size of each grid
        Mat cam0CurrImg = cam0CurrImgMsg.image;
        int grid_height = cam0CurrImg.rows() / processorConfig.gridRow;
        int grid_width = cam0CurrImg.cols() / processorConfig.gridCol;

        // Compute a rough relative rotation which takes a vector
        // from the previous frame to the current frame.
        Matx33f cam0Rpc;
        integrateImuData(cam0Rpc);

        // Organize the features in the previous image.
        List<Integer> prevIds = new ArrayList<>();
        List<Integer> prevLifetime = new ArrayList<>();
        MatOfPoint2f prevCam0Points = new MatOfPoint2f();
        List<Point> prevCam0PtsList = prevCam0Points.toList(); // TODO: make sure prevCam0Points is converted back into Mat if necessary
        for (Map.Entry<Integer, List<FeatureMetaData>> gridFeatures : prevFeatures.entrySet()) {
            for (FeatureMetaData prevFeature : gridFeatures.getValue()) {
                prevIds.add(prevFeature.id);
                prevLifetime.add(prevFeature.lifetime);
                prevCam0PtsList.add(prevFeature.cam0_point);
        }

        // Number of the features before tracking.
        int before_tracking = prevCam0PtsList.size();

        // Abort tracking if there is no features in
        // the previous frame.
        if (prevIds.isEmpty()) return;

        // Track features using LK optical flow method.
        // TODO: convert to list and then later call .fromList on them
        MatOfPoint2f currCam0Points;
        MatOfByte trackInliers = new MatOfByte();

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
        currFeatures.put(); // TODO: add to map
        // TODO
    }

    // TODO: replace all the Vec3f etc. with Mat, in cases where Mat needed to be casted to Vec3f as a result?
    // TODO: Alternatively: instead of casting, create a .fromMat(Mat) method
    public void integrateImuData(Mat cam0Rpc) {
        assert(Matx33f.isMatx33f(cam0Rpc));

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
}
