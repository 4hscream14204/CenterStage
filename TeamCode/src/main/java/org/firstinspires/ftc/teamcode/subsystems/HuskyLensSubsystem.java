package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

public class HuskyLensSubsystem extends SubsystemBase {

    public enum PropPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }

     HuskyLens huskyLens;




     public HuskyLensSubsystem(HuskyLens huskyLensConstructor) {
         huskyLens = huskyLensConstructor;
     }

     //TEST LOCATION FOR LEFT COORDINATE EQUALS 50
     //TEST LOCATION FOR RIGHT COORDINATE EQUALS 200

     public PropPosition getLocation (int intRightDetectionLine, int intLeftDetectionLine, RobotBase.Alliance alliance) {
         HuskyLens.Block[] blocks = huskyLens.blocks();
         for (int i = 0; i < blocks.length; i++) {
             int blockLeftCoordinate = blocks[i].left;
             if (blockLeftCoordinate > intRightDetectionLine) {
                 return PropPosition.RIGHT;
             } else if (blockLeftCoordinate < intLeftDetectionLine) {
                 return PropPosition.LEFT;
             } else {
                 return PropPosition.MIDDLE;
             }
         }
         return PropPosition.NONE;
     }
    }