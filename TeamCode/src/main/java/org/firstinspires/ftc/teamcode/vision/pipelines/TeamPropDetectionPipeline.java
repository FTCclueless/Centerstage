package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class TeamPropDetectionPipeline implements VisionProcessor {

    static int YCRCB_CHANNEL_IDX = 1;

    Rect leftRegion = new Rect(30,215,95,25);
    Rect centerRegion = new Rect(235,215,160,25);
    Rect rightRegion = new Rect(490,215,85,25);

    Mat leftMat, centerMat, rightMat = new Mat();
    Mat cbMat = new Mat();
    Mat deNoiseMat = new Mat();

    public enum TeamPropLocation {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    public TeamPropLocation team_prop_location = TeamPropLocation.LEFT; // default is center

    public double leftAvg, centerAvg, rightAvg;

    Telemetry telemetry;
    boolean isRed = true;

    public TeamPropDetectionPipeline (Telemetry telemetry, boolean isRed) {
        this.telemetry = telemetry;
        this.isRed = isRed;

        if (isRed) {
            YCRCB_CHANNEL_IDX = 1;
        } else {
            YCRCB_CHANNEL_IDX = 2;
        }
    }

    public VisionProcessor getVisionProcessor() {
        return this;
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

//        // cropping into 3 regions
        leftMat = deNoiseMat.submat(leftRegion);
        centerMat = deNoiseMat.submat(centerRegion);
        rightMat = deNoiseMat.submat(rightRegion);

        // averaging each region
        leftAvg = Core.mean(leftMat).val[0];
        centerAvg = Core.mean(centerMat).val[0];
        rightAvg = Core.mean(rightMat).val[0];

        // finding which region has greatest average
        if (leftAvg > centerAvg && leftAvg > rightAvg) {
            team_prop_location = TeamPropLocation.LEFT;
        } else if (centerAvg > leftAvg && centerAvg > rightAvg) {
            team_prop_location = TeamPropLocation.CENTER;
        } else {
            team_prop_location = TeamPropLocation.RIGHT;
        }

        return deNoiseMat;
    }

    public TeamPropLocation getTeamPropLocation() {
        return team_prop_location;
    }

    public void sendTeamPropTelemetry(Telemetry telemetry) {
        telemetry.addData("leftAvg", leftAvg);
        telemetry.addData("centerAvg", centerAvg);
        telemetry.addData("rightAvg", rightAvg);
        telemetry.addData("team prop location", getTeamPropLocation() + "");
        telemetry.update();
    }

    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(4, 4));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));

    private void noiseReduction(Mat input, Mat output)
    {
        Imgproc.erode(input, output, erodeElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint();
        nonSelectedPaint.setColor(Color.RED);
        nonSelectedPaint.setStyle(Paint.Style.STROKE);
        nonSelectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        switch (team_prop_location) {
            case LEFT:
                canvas.drawRect(makeGraphicsRect(leftRegion, scaleBmpPxToCanvasPx), selectedPaint);
                canvas.drawRect(makeGraphicsRect(centerRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                canvas.drawRect(makeGraphicsRect(rightRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                break;
            case CENTER:
                canvas.drawRect(makeGraphicsRect(leftRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                canvas.drawRect(makeGraphicsRect(centerRegion, scaleBmpPxToCanvasPx), selectedPaint);
                canvas.drawRect(makeGraphicsRect(rightRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(makeGraphicsRect(leftRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                canvas.drawRect(makeGraphicsRect(centerRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                canvas.drawRect(makeGraphicsRect(rightRegion, scaleBmpPxToCanvasPx), selectedPaint);
                break;
            case NONE:
                canvas.drawRect(makeGraphicsRect(leftRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                canvas.drawRect(makeGraphicsRect(centerRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                canvas.drawRect(makeGraphicsRect(rightRegion, scaleBmpPxToCanvasPx), nonSelectedPaint);
                break;
        }
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }
}