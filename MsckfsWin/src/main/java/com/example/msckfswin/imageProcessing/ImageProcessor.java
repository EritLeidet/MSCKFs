package com.example.msckfswin.imageProcessing;

import static org.opencv.core.Core.BORDER_CONSTANT;
import static org.opencv.core.Core.BORDER_REFLECT_101;
import static org.opencv.core.CvType.CV_8U;

import com.example.msckfswin.imuProcessing.ImuMessage;
import com.example.msckfswin.utils.Matx33d;
import com.example.msckfswin.utils.Matx33f;
import com.example.msckfswin.utils.Vec3d;
import com.example.msckfswin.utils.Vec3f;
import com.example.msckfswin.utils.Vec4d;
import com.example.msckfswin.utils.Vecd;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDouble;
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

import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.concurrent.SubmissionPublisher;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class ImageProcessor {


    private final ProcessorConfig processorConfig;

    // Indicate if this is the first image message.
    private boolean isFirstImg;

    // ID for the next new feature.
    private int next_feature_id;

    // Feature detector
    private final FastFeatureDetector featureDetector;

    // IMU message buffer
    private final List<ImuMessage> imuMsgBuffer = Collections.synchronizedList(new ArrayList<>()); // TODO: max-Capacity? Maybe replace with a Buffer datatype?


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
    private final MatOfInt cam0Resolution; // Vec2
    private final MatOfDouble cam0Intrinsics; // Vec4d
    private final String cam0DistortionModel;
    private final MatOfDouble cam0DistortionCoeffs; // Vec4d
    private Mat TCam0Imu;
    private Mat RCam0Imu;
    private Mat tCam0Imu;

    private final SubmissionPublisher<FeatureMessage> featurePublisher;

    public ImageProcessor(ProcessorConfig processorConfig, SubmissionPublisher<FeatureMessage> featurePublisher) {
        this.processorConfig = processorConfig;
        this.isFirstImg = true;
        this.next_feature_id = 0;
        this.featureDetector = FastFeatureDetector.create(processorConfig.fastThreshold);
        this.cam0CurrImgMsg = null;
        this.cam0PrevImgMsg = null;
        this.featurePublisher = featurePublisher;

        cam0Resolution = processorConfig.cam0Resolution;
        cam0Intrinsics = processorConfig.cam0Intrinsics;
        cam0DistortionModel = processorConfig.cam0DistortionModel;
        cam0DistortionCoeffs = processorConfig.cam0DistortionCoeffs;

    }


    public void imageCallback(final ImageMessage cam0imageMsg) {
        double startTime;

        // Get the current image.
        cam0CurrImgMsg = cam0imageMsg;
        createImagePyramids();


        // Detect features in the first frame.
        if (isFirstImg) {
            startTime = Instant.now().getEpochSecond();

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

        // Initialize the current features to empty vectors.
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
            gridFeatures.subList(processorConfig.gridMaxFeatureNum, gridFeatures.size()).clear();
            assert (gridFeatures.size() == processorConfig.gridMaxFeatureNum); // TODO: remove after successful run

        }

    }

    public void addNewFeatures() {
        final Mat currImg = cam0CurrImgMsg.image;

        // Size of each grid.
        int gridHeight = currImg.rows() / processorConfig.gridRow;
        int gridWidth = currImg.cols() / processorConfig.gridCol;

        // Create a mask to avoid redetecting existing features.
        Mat mask = new Mat(currImg.rows(), currImg.cols(), CV_8U, new Scalar(1));

        for (Map.Entry<Integer, List<FeatureMetaData>> gridFeatures : currFeatures.entrySet()) {
            for (FeatureMetaData feature : gridFeatures.getValue()) {
                final int x = (int) feature.cam0_point.x;
                final int y = (int) feature.cam0_point.y;

                int upLim = y - 2, bottomLim = y + 3, leftLim = x - 2, rightLim = x + 3;
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
        featureDetector.detect(currImg, newFeatures, mask);
        List<KeyPoint> newFeatList = newFeatures.toList();

        // Collect the new detected features based on the grid.
        // Select the ones with top response within each grid afterwards
        List<List<KeyPoint>> newFeatureSieve = Stream.generate(() -> new ArrayList<KeyPoint>()).limit((long) processorConfig.gridRow * processorConfig.gridCol).collect(Collectors.toList()); // List of empty lists.

        for (KeyPoint feature : newFeatList) {
            int row = (int) feature.pt.y / gridHeight;
            int col = (int) feature.pt.x / gridWidth;
            newFeatureSieve.get(row * processorConfig.gridCol + col).add(feature);
        }

        newFeatList.clear();
        for (List<KeyPoint> item : newFeatureSieve) {
            if (item.size() > processorConfig.gridMaxFeatureNum) {
                item.sort((kp0, kp1) -> Float.compare(kp1.response, kp0.response)); // descending order
                item.subList(processorConfig.gridMaxFeatureNum, item.size()).clear();

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
        for (int code = 0; code < processorConfig.gridRow * processorConfig.gridCol; code++) {
            List<FeatureMetaData> featuresThisGrid = currFeatures.get(code);
            List<FeatureMetaData> newFeaturesThisGrid = gridNewFeatures.get(code);

            if (featuresThisGrid.size() >= processorConfig.gridMinFeatureNum) continue;

            int vacancyNum = processorConfig.gridMinFeatureNum - featuresThisGrid.size();
            for (int k = 0; k < vacancyNum && vacancyNum < newFeaturesThisGrid.size(); k++) {
                featuresThisGrid.add(newFeaturesThisGrid.get(k));
                featuresThisGrid.get(featuresThisGrid.size() - 1).id = next_feature_id++;
                featuresThisGrid.get(featuresThisGrid.size() - 1).lifetime = 1;

                newAddedFeatureNum++;
            }
        }

    }

    /**
     * Group features into grid and sort features descending by response
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

            FeatureMetaData newFeature = new FeatureMetaData(cam0Point);
            newFeature.response = response;
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
        List<KeyPoint> newFeatList = newFeatures.toList();
        List<Point> cam0Points = newFeatList.stream().map(keyPoint -> keyPoint.pt).collect(Collectors.toList());

        // (skipped)

        // Group the features into grids
        GridFeatures gridNewFeatures = groupFeatures(cam0Points, newFeatList, gridHeight, gridWidth);

        // Collect new features within each grid with high response.
        for (int code = 0; code < processorConfig.gridRow * processorConfig.gridCol; code++) {
            List<FeatureMetaData> featuresThisGrid = currFeatures.get(code);
            List<FeatureMetaData> newFeaturesThisGrid = gridNewFeatures.get(code);

            for (int k = 0; k < processorConfig.gridMinFeatureNum && k < newFeaturesThisGrid.size(); k++) {
                featuresThisGrid.add(newFeaturesThisGrid.get(k));
                featuresThisGrid.get(featuresThisGrid.size() - 1).id = next_feature_id++;
                featuresThisGrid.get(featuresThisGrid.size() - 1).lifetime = 1;
            }
        }

    }


    public void predictFeatureTracking(final MatOfPoint2f inputPts, final Mat Rpc, final MatOfDouble intrinsics, MatOfPoint2f compensatedPts) {
        assert (Matx33f.isMatx33f(Rpc));
        assert (Vec4d.isVec4d(intrinsics));
        List<Point> ptsList = inputPts.toList();
        List<Point> compPtsList = compensatedPts.toList();
        // Return directly if there are no input features.
        if (ptsList.isEmpty()) {
            compPtsList.clear();
            return;
        }
        compPtsList = compPtsList.subList(0, ptsList.size());

        // Intrinsic matrix.
        Mat K = getK(intrinsics);
        Mat H = K.matMul(Rpc).matMul(K.inv());

        for (int i = 0; i < ptsList.size(); i++) {
            MatOfDouble p1 = Vec3d.create(ptsList.get(i).x, ptsList.get(i).y, 1.0d); // Vec3d instead of Vec3f
            Mat p2 = H.matMul(p1);
            assert (Vec3d.isVec3d(p2));
            compPtsList.get(i).x = Vecd.get(p2, 0) / Vecd.get(p2, 2);
            compPtsList.get(i).y = Vecd.get(p2, 1) / Vecd.get(p2, 2);

        }

        compensatedPts.fromList(compPtsList);

    }

    private Mat getK(final MatOfDouble intrinsics) {
        return Matx33d.create(new double[]{
                Vecd.get(intrinsics, 0), 0.0d, Vecd.get(intrinsics, 2),
                0.0d, Vecd.get(intrinsics, 1), Vecd.get(intrinsics, 3),
                0.0d, 0.0d, 1.0d});
    }


    public void createImagePyramids() {
        final Mat curr_cam0_img = cam0CurrImgMsg.image;
        Video.buildOpticalFlowPyramid(curr_cam0_img, currCam0Pyramid, new Size(processorConfig.patchSize, processorConfig.patchSize), processorConfig.pyramidLevels, true, BORDER_REFLECT_101, BORDER_CONSTANT, false);
    }


    public void publish() {
        // Publish features.

        List<Integer> currIds = new ArrayList<>();
        List<Point> currPointsList = new ArrayList<>();

        for (Map.Entry<Integer, List<FeatureMetaData>> gridFeatures : currFeatures.entrySet()) {
            for (FeatureMetaData feature : gridFeatures.getValue()) {
                currIds.add(feature.id);
                currPointsList.add(feature.cam0_point);
            }
        }

        MatOfPoint2f currCam0Points = new MatOfPoint2f();
        currCam0Points.fromList(currPointsList);
        MatOfPoint2f currCam0PointsUndistorted = new MatOfPoint2f();
        undistortPoints(currCam0Points, cam0Intrinsics, cam0DistortionModel, cam0DistortionCoeffs, currCam0PointsUndistorted);
        List<Point> currPointsUndistortedList = currCam0PointsUndistorted.toList();

        List<FeatureMeasurement> features = new ArrayList<>();
        for (int i = 0; i < currIds.size(); i++) {
            features.add(new FeatureMeasurement(currIds.get(i), currPointsUndistortedList.get(i).x, currPointsUndistortedList.get(i).y));
        }

        FeatureMessage featureMessage = new FeatureMessage(cam0CurrImgMsg.timestamp, features);
        featurePublisher.submit(featureMessage);

        // Publish tracking info.


    }

    public void undistortPoints(final MatOfPoint2f ptsIn, final MatOfDouble intrinsics, final String distortionModel, final MatOfDouble distortionCoeffs, final MatOfPoint2f ptsOut, final Mat rectificationMatrix, final MatOfDouble newIntrinsics) {
        assert (Vec4d.isVec4d(intrinsics));
        assert (Vec4d.isVec4d(newIntrinsics));
        assert (Vec4d.isVec4d(distortionCoeffs));
        assert (Matx33d.isMatx33d(rectificationMatrix));

        if (ptsIn.total() == 0) return;

        final Mat K = getK(intrinsics);
        final Mat newK = getK(newIntrinsics);

        if (distortionModel.equals("equidistant")) {
            Calib3d.fisheye_undistortPoints(ptsIn, K, distortionCoeffs, rectificationMatrix, newK);
        } else {
            Calib3d.undistortPoints(ptsIn, ptsOut, K, distortionCoeffs, rectificationMatrix, newK);
        }

    }

    public void undistortPoints(final MatOfPoint2f ptsIn, final MatOfDouble intrinsics, final String distortionModel, final MatOfDouble distortionCoeffs, final MatOfPoint2f ptsOut) {
        undistortPoints(ptsIn, intrinsics, distortionModel, distortionCoeffs, ptsOut, Matx33d.eye(), Vec4d.createVec4d(1, 1, 0, 0));
    }

    public void trackFeatures() {
        // Size of each grid
        int grid_height = cam0CurrImgMsg.image.rows() / processorConfig.gridRow;
        int grid_width = cam0CurrImgMsg.image.cols() / processorConfig.gridCol;

        // Compute a rough relative rotation which takes a vector
        // from the previous frame to the current frame.
        Mat cam0Rpc = Matx33f.create();
        cam0Rpc = integrateImuData(cam0Rpc);

        // Organize the features in the previous image.
        List<Integer> prevIds = new ArrayList<>();
        List<Integer> prevLifetime = new ArrayList<>();
        MatOfPoint2f prevCam0Points = new MatOfPoint2f();

        List<Point> prevCam0PtsList = prevCam0Points.toList();
        for (Map.Entry<Integer, List<FeatureMetaData>> gridFeatures : prevFeatures.entrySet()) {
            for (FeatureMetaData prevFeature : gridFeatures.getValue()) {
                prevIds.add(prevFeature.id);
                prevLifetime.add(prevFeature.lifetime);
                prevCam0PtsList.add(prevFeature.cam0_point);
            }
        }
        prevCam0Points.fromList(prevCam0PtsList);

        // Number of the features before tracking.
        int before_tracking = prevCam0PtsList.size();

        // Abort tracking if there is no features in
        // the previous frame.
        if (prevIds.isEmpty()) return;

        // Track features using LK optical flow method.
        MatOfPoint2f currCam0Points = new MatOfPoint2f();
        MatOfByte trackInliers = new MatOfByte();
        predictFeatureTracking(prevCam0Points, cam0Rpc, processorConfig.cam0Intrinsics, currCam0Points);
        Video.calcOpticalFlowPyrLK(cam0PrevImgMsg.image, cam0CurrImgMsg.image, prevCam0Points, currCam0Points, trackInliers, new MatOfFloat(), new Size(processorConfig.patchSize, processorConfig.patchSize), processorConfig.pyramidLevels, new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, processorConfig.maxIteration, processorConfig.trackPrecision), Video.OPTFLOW_USE_INITIAL_FLOW);

        // Mark those tracked points out of the image region
        // as untracked.
        List<Point> currCam0PtsList = currCam0Points.toList();
        List<Byte> trackInList = trackInliers.toList();
        for (int i = 0; i < trackInList.size(); i++) {
            if (trackInList.get(i) == 0) continue;
            if (currCam0PtsList.get(i).y < 0 || currCam0PtsList.get(i).y > cam0CurrImgMsg.image.rows() - 1 || currCam0PtsList.get(i).x < 0 || currCam0PtsList.get(i).x > cam0CurrImgMsg.image.cols() - 1) {
                trackInList.set(i, (byte) 0);
            }
        }

        // Collect the tracked points.
        List<Integer> prevTrackedIds = new ArrayList<>();
        List<Integer> prevTrackedLifetime = new ArrayList<>();
        List<Point> prevTrackedCam0Points = new ArrayList<>();
        List<Point> currTrackedCam0Points = new ArrayList<>();


        removeUnmarkedElements(prevIds, trackInList, prevTrackedIds);
        removeUnmarkedElements(prevLifetime, trackInList, prevTrackedLifetime);
        removeUnmarkedElements(prevCam0PtsList, trackInList, prevTrackedCam0Points);
        removeUnmarkedElements(currCam0PtsList, trackInList, currTrackedCam0Points);

        // Number of features left after tracking.
        int afterTracking = currTrackedCam0Points.size();

        // Step 1: stereo matching. (skipped)
        List<Point> currMatchedCam0Points = currTrackedCam0Points; // TODO: replace CurrMatched with currTracked if correct.
        List<Point> prevMatchedCam0Points = prevTrackedCam0Points;

        // Step 2: // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1.
        List<Integer> cam0RansacInliers;

        cam0RansacInliers = twoPointRansac(prevMatchedCam0Points);

        // Number of features after ransac.
        int after_ransac = 0;

        for (int i = 0; i < cam0RansacInliers.size(); ++i) {
            if (cam0RansacInliers.get(i) == 0) continue;
            int row = (int) currMatchedCam0Points.get(i).y / grid_height;
            int col = (int) currMatchedCam0Points.get(i).x / grid_width;
            int code = row*processorConfig.gridCol + col;
            FeatureMetaData newFeature = new FeatureMetaData(currMatchedCam0Points.get(i));
            newFeature.id = prevTrackedIds.get(i);
            newFeature.lifetime = prevTrackedLifetime.get(i) + 1;
            prevTrackedLifetime.set(i, prevTrackedLifetime.get(i) + 1);
            currFeatures.get(code).add(newFeature);

            after_ransac++;

        } //for

        // Compute the tracking rate. (skipped)
    }

    public Mat integrateImuData(Mat cam0Rpc) {
        assert (Matx33f.isMatx33f(cam0Rpc));

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
        MatOfFloat mean_ang_vel = Vec3f.create();
        for (int i = beginIter; i < endIter; i++) {
            Core.add(mean_ang_vel, imuMsgBuffer.get(i).angularVelocity, mean_ang_vel); // Python
        }


        if (endIter - beginIter > 0) {
            Core.multiply(mean_ang_vel, new Scalar(1.0f / (endIter - beginIter)), mean_ang_vel);
        }

        // Transform the mean angular velocity from the IMU
        // frame to the cam0 frames.
        Mat cam0MeanAngVel = RCam0Imu.t().matMul(mean_ang_vel);
        assert(Vec3f.isVec3f(cam0MeanAngVel));

        // Compute the relative rotation.
        double dtime = cam0CurrImgMsg.timestamp - cam0PrevImgMsg.timestamp;

        Mat scaledAngVel = new Mat();
        Core.multiply(cam0MeanAngVel, new Scalar(dtime), scaledAngVel); //
        Calib3d.Rodrigues(scaledAngVel, cam0Rpc);
        cam0Rpc = cam0Rpc.t();
        assert (Matx33f.isMatx33f(cam0Rpc));


        // Delete the useless and used imu messages.
        imuMsgBuffer.subList(0, endIter).clear();

        return cam0Rpc;

    }

    /**
     * // TODO: JavaDoc
     * Remove the unmarked elements within a vector.
     *
     * @Param
     */
    public <T> void removeUnmarkedElements(final List<T> rawVec, final List<Byte> markers, List<T> refinedVec) {
        assert (rawVec.size() == markers.size());
        for (int i = 0; i < markers.size(); i++) {
            if (markers.get(i) == 0) continue;
            refinedVec.add(rawVec.get(i));
        }
    }

    public List<Integer> twoPointRansac(List<Point> prevMatchedCam0Points) {
        // STUB
        return Stream.generate(() -> 1).limit(prevMatchedCam0Points.size()).collect(Collectors.toList());
        // TODO: properly RANSAC. Python commented this method out completely.
    }


}
