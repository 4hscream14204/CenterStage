package org.firstinspires.ftc.teamcode.subsystems;

import +com.qualccom.hardware.dfrobot.HuskLens;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

public class huskyLensSubsystem extends SubsystemBase {

private final int READ_PERIOD = 100;

private HuskyLens huskyLens;

 public void runOpMode()
 {
    huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

Deadline ratelimit = new Deadline(READ_PERIOD, TimeUnit.MILLISECONDS);

ratelimit.expire();

if (!huskyLens.knock()){
    telemetry.addData("problem communicating with" + huskyLens.getDeviceName());
} else {
    telemetry.addData("Press Start to continue");
}
huskyLens.selectAlgorithm(HuskyLens.Algorithe.TAG_RECOGNITION);

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
}
}