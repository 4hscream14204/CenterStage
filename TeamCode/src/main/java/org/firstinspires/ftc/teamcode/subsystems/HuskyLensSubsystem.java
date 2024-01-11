package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HuskyLensSubsystem extends SubsystemBase {

    //Display: 2.0-inch IPS screen with 320*240 resolution
     HuskyLens huskyLens;
     private RobotBase.PropPosition propPosition;
     private RobotBase.StartPosition startPosition;
     private int intAllianceNumber = 1;
     private int intLeftDetectionLine = 110;
    private int intRightDetectionLine = 300;
    private double dblLastDetectTime = System.currentTimeMillis();
    private double dblDetectionThreshold = 2000;

     public HuskyLensSubsystem(HuskyLens huskyLensConstructor) {
         huskyLens = huskyLensConstructor;
         propPosition = RobotBase.PropPosition.NONE;
     }


     public RobotBase.PropPosition getLocation (RobotBase.Alliance alliance, RobotBase.StartPosition startPosition) {
         HuskyLens.Block[] blocks = huskyLens.blocks();
         huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
         if (startPosition == RobotBase.StartPosition.LEFT) {
             intLeftDetectionLine = 110;
             intRightDetectionLine = 300;
         } else {
             intLeftDetectionLine = 5;
             intRightDetectionLine = 155;
         }
         if (alliance == RobotBase.Alliance.RED) {
             intAllianceNumber = 1;
         } else {
             intAllianceNumber = 2;
         }

         if((System.currentTimeMillis() - dblLastDetectTime) > dblDetectionThreshold) {
             if(startPosition == RobotBase.StartPosition.LEFT){
                 propPosition = RobotBase.PropPosition.RIGHT;
             } else {
                 propPosition = RobotBase.PropPosition.LEFT;
             }
         }

         for (int i = 0; i < blocks.length; i++) {
             dblLastDetectTime = System.currentTimeMillis();
             int blockLeftCoordinate = blocks[i].left;
             if (blocks[i].id == intAllianceNumber) {
                 if (startPosition == RobotBase.StartPosition.LEFT) {
                     if (blockLeftCoordinate < intLeftDetectionLine) {
                         propPosition = RobotBase.PropPosition.LEFT;
                     } else if (blockLeftCoordinate > intLeftDetectionLine && blockLeftCoordinate < intRightDetectionLine) {
                         propPosition = RobotBase.PropPosition.MIDDLE;
                     } else {
                         propPosition = RobotBase.PropPosition.RIGHT;
                     }
             } else {
                     if (blockLeftCoordinate > intRightDetectionLine) {
                         propPosition = RobotBase.PropPosition.RIGHT;
                     } else if (blockLeftCoordinate > intLeftDetectionLine && blockLeftCoordinate < intRightDetectionLine) {
                         propPosition = RobotBase.PropPosition.MIDDLE;
                     } else {
                         propPosition = RobotBase.PropPosition.LEFT;
                     }
                 }
             }
         }
         return propPosition;
     }
    }