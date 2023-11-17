package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

@TeleOp(name="HuskyLensTest")
public class TeleHuskyLensTest extends OpMode {

    RobotBase robotBase = new RobotBase(hardwareMap);

    int blockTopCoordinate;
    int blockLeftCoordinate;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

        //read in any objects into an array of blocks
        HuskyLens.Block[] blocks = robotBase.huskyLens.blocks();

//Loop through the array of blocks to get data from an individual block

        for (int i = 0;i<blocks.length;i++)

        {
            //Assign the block attributes to variables to use later
            blockTopCoordinate = blocks[i].top;
            blockLeftCoordinate = blocks[i].left;

            //Display the data on the driver hub using telemetry
            telemetry.addData("Block", blocks[i].toString());
            telemetry.addData("Block Top", blockTopCoordinate);
            telemetry.addData("Block Left", blockLeftCoordinate);
            telemetry.addData("Block ID", blocks[i].id);
            telemetry.addData("Block Height", blocks[i].height);
            telemetry.addData("Block Width", blocks[i].width);

        }
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }


}