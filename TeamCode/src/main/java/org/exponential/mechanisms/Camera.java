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

public class Camera implements Mechanism {
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
        camera.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);

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
        public Mat processFrame(Mat input){   // http://creativemorphometrics.co.vu/blog/2014/08/05/automated-outlines-with-opencv-in-python/

            // cropping image
            // TODO: change the cropping dimensions
            Rect crop = new Rect(new Point(0, 0), new Point(1280, 720));
            croppedImage = new Mat(input, crop);

            // Turn image to black and white
            Imgproc.cvtColor(input, blackAndWhite, Imgproc.COLOR_BGR2GRAY);

            // Get the "binary image" using a color threshold
            // https://stackoverflow.com/questions/31289895/threshold-image-using-opencv-java
            // TODO: change thresh (15) val to something during testing
            Imgproc.threshold(blackAndWhite, thresholded, 15, 225, Imgproc.THRESH_BINARY);
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

            return matrix;
        }

    }
}
