package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HuskyLensSubsystem extends SubsystemBase {


     HuskyLens huskyLens;




     public HuskyLensSubsystem(HuskyLens huskyLensConstructor) {
         huskyLens = huskyLensConstructor;
     }


     public RobotBase.PropPosition getLocation (int intRightDetectionLine, int intLeftDetectionLine, RobotBase.Alliance alliance, RobotBase.StartPosition startPosition) {
         HuskyLens.Block[] blocks = huskyLens.blocks();
         if (startPosition == RobotBase.StartPosition.LEFT) {
             intLeftDetectionLine = 100;
             intRightDetectionLine = 250;
         } else {
             intLeftDetectionLine = 50;
             intRightDetectionLine = 200;
         }
         for (int i = 0; i < blocks.length; i++) {
             int blockLeftCoordinate = blocks[i].left;
             if (blockLeftCoordinate > intRightDetectionLine) {
                 return RobotBase.PropPosition.RIGHT;
             } else if (blockLeftCoordinate < intLeftDetectionLine) {
                 return RobotBase.PropPosition.LEFT;
             } else {
                 return RobotBase.PropPosition.MIDDLE;
             }
         }
         return RobotBase.PropPosition.NONE;
     }
    }