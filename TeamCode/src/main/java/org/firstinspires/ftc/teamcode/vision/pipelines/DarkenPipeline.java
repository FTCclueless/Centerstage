package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

public class DarkenPipeline implements VisionProcessor {

    public DarkenPipeline() {}
    public VisionProcessor getVisionProcessor() {
        return this;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        //Creating an empty matrix
        Mat dest = new Mat(input.rows(), input.cols(), input.type());
        //Increasing the brightness of an image
        input.convertTo(dest, -1, 1, 1000);

        return dest;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.BLUE);
        canvas.drawRect(new Rect(235,285,160,50), selectedPaint);
    }
}