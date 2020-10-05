package org.exponential.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.exponential.superclasses.Mechanism;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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
        public Mat processFrame(Mat input){


            double brightnessMin = Double.MAX_VALUE;
            double brightnessAvg = 0;

            //For each segment
            numRings = -1;

            return matrix;
        }

    }
}
