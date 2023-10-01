package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;

import androidx.annotation.VisibleForTesting;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class TeamPropDetectionPipeline implements VisionProcessor {

    static final int YCRCB_CHANNEL_IDX = 2;

    Rect leftRegion = new Rect(0,50,50,50);
    Rect centerRegion = new Rect(50,50,50,50);
    Rect rightRegion = new Rect(100,50,50,50);

    Mat leftMat, centerMat, rightMat = new Mat();
    Mat cbMat = new Mat();
    Mat deNoiseMat = new Mat();

    enum TEAM_PROP_LOCATION {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public TEAM_PROP_LOCATION team_prop_location = TEAM_PROP_LOCATION.NONE; // default is center

    double leftAvg, centerAvg, rightAvg;

    Telemetry telemetry;

    public TeamPropDetectionPipeline (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        // converting color space to YCRCB
        Imgproc.cvtColor(input, cbMat, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(cbMat, cbMat, YCRCB_CHANNEL_IDX);

        // dilating and eroding to remove noise
        noiseReduction(cbMat, deNoiseMat);

        // cropping into 3 regions
        leftMat = deNoiseMat.submat(leftRegion);
        centerMat = deNoiseMat.submat(centerRegion);
        rightMat = deNoiseMat.submat(rightRegion);

        // averaging each region
        leftAvg = Core.mean(leftMat).val[0];
        centerAvg = Core.mean(centerMat).val[0];
        rightAvg = Core.mean(rightMat).val[0];

        // finding which region has greatest average
        if (leftAvg > centerAvg && leftAvg > rightAvg) {
            team_prop_location = TEAM_PROP_LOCATION.LEFT;
        } else if (centerAvg > leftAvg && centerAvg > rightAvg) {
            team_prop_location = TEAM_PROP_LOCATION.CENTER;
        } else {
            team_prop_location = TEAM_PROP_LOCATION.RIGHT;
        }

        // drawing rectangles
        Imgproc.rectangle(input, leftRegion, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(input, centerRegion, new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(input, rightRegion, new Scalar(0, 255, 0), 4);

        // sending telemetry
        telemetry.addData("team_prop_location", team_prop_location);
        telemetry.update();

        return null;
    }

    public TEAM_PROP_LOCATION getTeamPropLocation() {
        return team_prop_location;
    }

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    private void noiseReduction(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}