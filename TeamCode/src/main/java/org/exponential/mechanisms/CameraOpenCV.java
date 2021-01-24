package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.superclasses.Mechanism;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CameraOpenCV implements Mechanism {
    OpenCvCamera camera;
    LinearOpMode opMode;

    int numRings;

    @Override
    public void initialize(LinearOpMode opMode) {
        this.opMode = opMode;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Open connection to camera
        camera.openCameraDevice();

        camera.setPipeline(new Pipeline());
    }


    public void activate(){
        camera.startStreaming(1280, 720, /*OpenCvCameraRotation.SIDEWAYS_LEFT*/OpenCvCameraRotation.UPRIGHT);

    }

    public void deactivate() {
        camera.stopStreaming();
    }

    public int getNumberOfRings() {
        return numRings;
    }

    class Pipeline extends OpenCvPipeline {
        Mat matrix = new Mat();
        Mat croppedImage = new Mat();
        Mat croppedImageUpper = new Mat();
        Mat croppedImageLower = new Mat();
        Mat HSVImage = new Mat();
        Mat mergedImage = new Mat();

        Mat maskedImage = new Mat();
        Mat blackAndWhite = new Mat();
        Mat thresholded = new Mat();
        Mat contourImage = new Mat();

        final Scalar BLUE = new Scalar(0, 0, 255);
        final Scalar GREEN = new Scalar(0, 255, 0);

        final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(160, 130);

        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 50;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines


            numRings = -1; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD) {
                numRings = 4;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                numRings = 1;
            } else{
                numRings = 0;
            }

            opMode.telemetry.addData("value",avg1 );
            opMode.telemetry.addData("rings", numRings);
            opMode.telemetry.update();


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }
        /*public Mat processFrame(Mat input){
            // Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            // cropping the input to the following dimensions
            // x: 0 -> 720
            // y: 0 -> 1280
            // (0, 0): top right
            // lets say the orange rings are around hex 184, 104, 0
            // if robot is located on right red strip, the rings are located near the top of the middle.
            Rect imageCrop = new Rect(new Point(250, 160), new Point(410, 440));
            croppedImage = new Mat(input, imageCrop);

            Rect upperCrop = new Rect(new Point(330,160), new Point(410, 440));
            croppedImageUpper = new Mat(input, upperCrop);

            Rect lowerCrop = new Rect(new Point(250, 160), new Point(330, 440));
            croppedImageLower = new Mat(input, lowerCrop);
            double avgR = 0;
            double avgG = 0;
            double avgB = 0;
            int numPixels = 0;

            Imgproc.cvtColor(croppedImage, HSVImage, Imgproc.COLOR_BGR2HSV);
            // https://stackoverflow.com/questions/48528754/what-are-recommended-color-spaces-for-detecting-orange-color-in-open-cv
            // h value: [8, 25]
            // s value: [100, 255]
            // v value: [20, 255]
            //Core.inRange(HSVImage, new Scalar(8, 100, 20), new Scalar(25, 255, 255), maskedImage);
            ArrayList<Mat> split = new ArrayList<Mat>();
            // from my understand, split gives an arraylist of 3 matrices, each that just contain scalars for each channel
            Core.split(HSVImage, split);
            // https://stackoverflow.com/questions/38998402/opencv-split-and-change-value-in-that-channel-in-android
            // add to saturation (S of HSV)
            Mat changedSChannel = new Mat();
            Core.add(split.get(1), new Scalar(50), changedSChannel);
            split.set(1, changedSChannel);
            // https://stackoverflow.com/questions/52107379/intensify-or-increase-saturation-of-an-image
            Core.merge(split, mergedImage);

            *//*for (int y = 0; y < croppedImage.rows(); y += 2) {
                for (int x = 0; x < croppedImage.cols(); x += 2) {
                    double[] rgb = croppedImage.get(y, x);  // NOTE:: apparently, the double[] returned by get is [r, g, b]
                    avgR += rgb[0];
                    avgG += rgb[1];
                    avgB += rgb[2];
                    numPixels++;
                }
            }
            avgR /= numPixels;
            avgG /= numPixels;
            avgB /= numPixels;
            // converting the cropped image to black and whtie and storing it in blackAndWhite
            // Imgproc.cvtColor(croppedImage, blackAndWhite, Imgproc.COLOR_BGR2GRAY);

            opMode.telemetry.addData("r", avgR);
            opMode.telemetry.addData("g", avgG);
            opMode.telemetry.addData("b", avgB);
            opMode.telemetry.addData("all averages", (avgR + avgG + avgB) / 3);
            opMode.telemetry.addData("numRings", numRings);
            opMode.telemetry.update();*//*

          *//*  for (int y = 0; y < croppedImageUpper.rows(); y += 2) {
                for (int x = 0; x < croppedImageUpper.cols(); x += 2) {
                    double[] rgb = croppedImageUpper.get(y, x);  // NOTE:: apparently, the double[] returned by get is [r, g, b]
                    avgR += rgb[0];
                    avgG += rgb[1];
                    avgB += rgb[2];
                    numPixels++;
                }
            }
            avgR /= numPixels;
            avgG /= numPixels;
            avgB /= numPixels;
            // converting the cropped image to black and whtie and storing it in blackAndWhite
            // Imgproc.cvtColor(croppedImage, blackAndWhite, Imgproc.COLOR_BGR2GRAY);

            opMode.telemetry.addData("r", avgR);
            opMode.telemetry.addData("g", avgG);
            opMode.telemetry.addData("b", avgB);
            opMode.telemetry.addData("all averages", (avgR + avgG + avgB) / 3);
            //opMode.telemetry.addData("numRings", numRings);
            //---------------------------------------------------------------------------------------------------------------------
            avgR = 0;
            avgB = 0;
            avgG = 0;
            for (int y = 0; y < croppedImageLower.rows(); y += 2) {
                for (int x = 0; x < croppedImageLower.cols(); x += 2) {
                    double[] rgb = croppedImageLower.get(y, x);  // NOTE:: apparently, the double[] returned by get is [r, g, b]
                    avgR += rgb[0];
                    avgG += rgb[1];
                    avgB += rgb[2];
                    numPixels++;
                }
            }
            avgR /= numPixels;
            avgG /= numPixels;
            avgB /= numPixels;
            // converting the cropped image to black and whtie and storing it in blackAndWhite
            // Imgproc.cvtColor(croppedImage, blackAndWhite, Imgproc.COLOR_BGR2GRAY);

            opMode.telemetry.addData("r", avgR);
            opMode.telemetry.addData("g", avgG);
            opMode.telemetry.addData("b", avgB);
            opMode.telemetry.addData("all averages", (avgR + avgG + avgB) / 3);
            //opMode.telemetry.addData("numRings", numRings);
            opMode.telemetry.update();*//*
            return mergedImage;  // what gets returned is showed on the robot phone screen
        }*/
       /* int threshold = 70;
        public Mat processFrame(Mat input){   // http://creativemorphometrics.co.vu/blog/2014/08/05/automated-outlines-with-opencv-in-python/

            // cropping image
            // TODO: change the cropping dimensions
            Rect crop = new Rect(new Point(0, 0), new Point(600, 600));
            croppedImage = new Mat(input, crop);

            // Turn image to black and white
            Imgproc.cvtColor(input, blackAndWhite, Imgproc.COLOR_BGR2GRAY);
            // Get the "binary image" using a color threshold
            // https://stackoverflow.com/questions/31289895/threshold-image-using-opencv-java
            // TODO: change thresh (70) val to something during testing
            if (opMode.gamepad1.dpad_up) {
                threshold += 1;
            } else if (opMode.gamepad1.dpad_down) {
                threshold -= 1;
            }
            Imgproc.threshold(blackAndWhite, thresholded, threshold, 225, Imgproc.THRESH_BINARY);
            ArrayList<MatOfPoint> listOfContours = new ArrayList<MatOfPoint>();

            // find contours from binary image, https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a
            // RETR_TREE: https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga819779b9857cc2f8601e6526a3a5bc71 (retrieval modes)
            // RETR_TREE and CHAIN_APPROX_SIMPLE from the article clink above
            Imgproc.findContours(thresholded, listOfContours, contourImage, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // TODO: Determine size of contour to see how many rings there are
            //  (if there are multiple contours, you gotta get rid of them somehow by (or by more cropping)
            numRings = -1;
            for (MatOfPoint contour: listOfContours) {
                double area = Imgproc.contourArea(contour);
                // do stuff with this :)
            }
            opMode.telemetry.addData("threshold", threshold);
            opMode.telemetry.update();
            return thresholded;
        }*/

    }
}
