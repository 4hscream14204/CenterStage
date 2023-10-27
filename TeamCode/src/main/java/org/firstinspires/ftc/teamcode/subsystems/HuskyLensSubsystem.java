package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.arcrobotics.ftclib.command.SubsystemBase;

public class HuskyLensSubsystem extends SubsystemBase {

    private final int READ_PERIOD = 100;

    private HuskyLens huskyLens;

    public void runOpMode() {
     /*
 }
    huskyLens = hwMap.get(HuskyLens.class, "huskylens");

Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);

rateLimit.expire();

if (!huskyLens.knock()){
    telemetry.addData("problem communicating with" + huskyLens.getDeviceName());
} else {
    telemetry.addData("Press Start to continue");
}
huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

telemetry.update();
waitForStart();

while(opModeIsActive()) {
    if (!rateLimit.hasExpired()) {
        continue;
    }
    rateLimit.reset();

    HuskyLens.Block[] blocks = huskyLens.blocks();
    telemetry.addData("Block Count", blocks.length);
    for (int i = 0; i < blocks.length; i++) {
        telemetry.addData("Block", blocks[i].toString());
    }
    telemetry.update();


      */
    }
}