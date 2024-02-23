package org.firstinspires.ftc.teamcode.subsystems;


/*import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class LogitechCameraSubsystem implements VisionProcessor {
    //since the camera is upside down, right is left and left is right
    //this is more efficient than rotating the camera orientation which has a lot of overhead
    public Rect rectRightSideRight = new Rect(10,70,150,230);
    public Rect rectMiddleSideRight = new Rect(190, 120, 340, 180);

    public  Rect rectMiddleSideLeft = new Rect(190, 120, 340, 180);
    public Rect rectLeftSideLeft = new Rect(610, 100, 170, 200);

    private RobotBase.PropPosition propPosition;

    private RobotBase.StartPosition startPosition;

    private VisionPortal visionPortal;

    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();



    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        startup();
    }

    public LogitechCameraSubsystem(RobotBase.StartPosition startPositionCon) {
        startPosition = startPositionCon;
    }
    

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeftSideLeft);
        double satRectMiddleSideLeft = getAvgSaturation(hsvMat, rectMiddleSideLeft);
        double satRectMiddleSideRight = getAvgSaturation(hsvMat, rectMiddleSideRight);
        double satRectRight = getAvgSaturation(hsvMat, rectRightSideRight);
        if (startPosition == RobotBase.StartPosition.LEFT) {
            if ((satRectLeft  > satRectMiddleSideLeft) && (satRectLeft > satRectRight)) {
                return Selected.LEFT;
            } else if ((satRectMiddleSideLeft > satRectLeft) && (satRectMiddleSideLeft > satRectRight)) {
                return Selected.MIDDLE;
            }
            return Selected.RIGHT;

        } else {
            if ((satRectRight > satRectMiddleSideRight) && (satRectRight > satRectRight)) {
                return Selected.RIGHT;
            } else if ((satRectMiddleSideRight > satRectLeft) && (satRectMiddleSideRight > satRectRight)) {
                return Selected.MIDDLE;
            }
        }
        return Selected.LEFT;



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

        android.graphics.Rect drawRectLeft = makeGraphicsRect(rectLeftSideLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectMiddle = makeGraphicsRect(rectMiddleSideRight, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectRight = makeGraphicsRect(rectRightSideRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectLeft, selectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, selectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
        }
        telemetry.update();
    }

    public Selected getSelection() {
        return selection;
    }

    public void startup (){
            visionPortal.resumeStreaming();

    }

    public enum Selected { NONE, LEFT, MIDDLE, RIGHT }
}*/