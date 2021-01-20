package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.superclasses.Mechanism;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class CameraOpenCVOld implements Mechanism {
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
        Mat blackAndWhite = new Mat();
        Mat thresholded = new Mat();
        Mat contourImage = new Mat();
       /* public Mat processFrame(Mat input){
            // Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            // cropping the input to the following dimensions
            // x: 0 -> 720
            // y: 0 -> 1280
            // (0, 0): top right
            // lets say the orange rings are around hex 184, 104, 0
            // if robot is located on right red strip, the rings are located near the top of the middle.
            Rect imageCrop = new Rect(new Point(250, 160), new Point(410, 440));
            croppedImage = new Mat(input, imageCrop);

            double avgR = 0;
            double avgG = 0;
            double avgB = 0;
            int numPixels = 0;
            for (int y = 0; y < croppedImage.rows(); y += 2) {
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
            opMode.telemetry.update();

            return croppedImage;  // what gets returned is showed on the robot phone screen
        }*/
        int threshold = 70;
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
        }

    }
}
