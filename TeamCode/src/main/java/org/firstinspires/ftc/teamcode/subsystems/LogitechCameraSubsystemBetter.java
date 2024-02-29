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
    public Rect rectRightSideRight = new Rect(320,350,130,130);
    public Rect rectMiddleSideRight = new Rect(360, 90, 110, 120);

    public  Rect rectMiddleSideLeft = new Rect(360, 310, 110, 120);
    public Rect rectLeftSideLeft = new Rect(330, 40, 130, 130);

    public Rect rectSide ;

    public Rect rectMiddle;

    public double satRectSide;

    public double satRectMiddle;

    private double colorThreshold = 70;



   public Selected selection = Selected.NONE;

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
        satRectSide = getAvgSaturation(hsvMat, rectSide);
        satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        /*double satRectMiddleSideRight = getAvgSaturation(hsvMat, rectMiddleSideRight);
        double satRectRight = getAvgSaturation(hsvMat, rectRightSideRight);

         */

            if (satRectSide  > colorThreshold)  {
                return Selected.SIDE;
            } else if (satRectMiddle > colorThreshold)  {
                return Selected.MIDDLE;
            }
            return Selected.NONE;


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

    public RobotBase.PropPosition getLocation () {
        RobotBase.PropPosition propPosition;
        if (startPosition == RobotBase.StartPosition.LEFT) {
            if (selection == Selected.SIDE) {
                propPosition = RobotBase.PropPosition.LEFT;
            } else if(selection == Selected.MIDDLE) {
                propPosition = RobotBase.PropPosition.MIDDLE;
            } else {
                propPosition = RobotBase.PropPosition.RIGHT;
            }
        } else {
            if (selection == Selected.SIDE) {
                propPosition = RobotBase.PropPosition.RIGHT;
            } else if(selection == Selected.MIDDLE) {
                propPosition = RobotBase.PropPosition.MIDDLE;
            } else {
                propPosition = RobotBase.PropPosition.LEFT;
            }
        }
        return propPosition;
    }

    public enum Selected { MIDDLE, SIDE, NONE }
}