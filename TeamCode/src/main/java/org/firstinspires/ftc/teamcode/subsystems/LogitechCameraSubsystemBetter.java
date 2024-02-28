package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class LogitechCameraSubsystemBetter implements VisionProcessor {
    //since the camera is upside down, right is left and left is right
    //this is more efficient than rotating the camera orientation which has a lot of overhead
    public Rect rectRightSideRight = new Rect(10,70,150,230);
    public Rect rectMiddleSideRight = new Rect(190, 120, 340, 180);

    public  Rect rectMiddleSideLeft = new Rect(190, 120, 340, 180);
    public Rect rectLeftSideLeft = new Rect(610, 100, 170, 200);

    public Rect rectSide ;

    public Rect rectMiddle;
    Selected selection = Selected.NONE;

    private RobotBase.StartPosition startPosition;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    public LogitechCameraSubsystemBetter(RobotBase.StartPosition startPositionCon) {
        startPosition = startPositionCon;
        if (startPosition == RobotBase.StartPosition.RIGHT){
            rectMiddle = rectMiddleSideRight;
            rectSide = rectRightSideRight;
        } else if (startPosition == RobotBase.StartPosition.LEFT) {
            rectMiddle = rectMiddleSideLeft;
            rectSide = rectLeftSideLeft;
        }


    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        /*double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            return Selected.MIDDLE;
        }
        return Selected.RIGHT;

         */
        double satRectLeft = getAvgSaturation(hsvMat, rectLeftSideLeft);
        double satRectMiddleSideLeft = getAvgSaturation(hsvMat, rectMiddleSideLeft);
        double satRectMiddleSideRight = getAvgSaturation(hsvMat, rectMiddleSideRight);
        double satRectRight = getAvgSaturation(hsvMat, rectRightSideRight);
        if (startPosition == RobotBase.StartPosition.LEFT) {
            if ((satRectLeft  > satRectMiddleSideLeft) && (satRectLeft > satRectRight)) {
                return LogitechCameraSubsystem.Selected.LEFT;
            } else if ((satRectMiddleSideLeft > satRectLeft) && (satRectMiddleSideLeft > satRectRight)) {
                return LogitechCameraSubsystem.Selected.MIDDLE;
            }
            return LogitechCameraSubsystem.Selected.RIGHT;

        } else {
            if ((satRectRight > satRectMiddleSideRight) && (satRectRight > satRectRight)) {
                return LogitechCameraSubsystem.Selected.RIGHT;
            } else if ((satRectMiddleSideRight > satRectLeft) && (satRectMiddleSideRight > satRectRight)) {
                return LogitechCameraSubsystem.Selected.MIDDLE;
            }
        }


        return LogitechCameraSubsystem.Selected.LEFT;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.RED);

        android.graphics.Rect drawRectSide = makeGraphicsRect(rectSide, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection) {
            case SIDE:
                canvas.drawRect(drawRectSide, selectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectSide, selectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectSide, selectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                break;
        }
    }

    public Selected getSelection() {
        return selection;
    }

    public enum Selected { MIDDLE, SIDE, NONE }
}