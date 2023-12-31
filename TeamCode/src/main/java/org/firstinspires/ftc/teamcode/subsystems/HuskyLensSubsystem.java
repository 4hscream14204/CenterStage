package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HuskyLensSubsystem extends SubsystemBase {


     HuskyLens huskyLens;
     private RobotBase.PropPosition propPosition;
     private int intAllianceNumber = 1;
     private int intLeftDetectionLine = 80;
    private int intRightDetectionLine = 280;

     public HuskyLensSubsystem(HuskyLens huskyLensConstructor) {
         huskyLens = huskyLensConstructor;
         propPosition = RobotBase.PropPosition.NONE;
     }


     public RobotBase.PropPosition getLocation (RobotBase.Alliance alliance, RobotBase.StartPosition startPosition) {
         HuskyLens.Block[] blocks = huskyLens.blocks();
         huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
         if (startPosition == RobotBase.StartPosition.LEFT) {
             intLeftDetectionLine = 100;
             intRightDetectionLine = 250;
         } else {
             intLeftDetectionLine = 80;
             intRightDetectionLine = 230;
         }
         if (alliance == RobotBase.Alliance.RED) {
             intAllianceNumber = 1;
         } else {
             intAllianceNumber = 2;
         }
         for (int i = 0; i < blocks.length; i++) {
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