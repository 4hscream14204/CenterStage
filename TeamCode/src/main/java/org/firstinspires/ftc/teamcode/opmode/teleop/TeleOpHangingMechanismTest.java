package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;


@TeleOp(name="HangingMechanismTest")
public class TeleOpHangingMechanismTest extends OpMode {

    public RobotBase robotBase;
    private GamepadEx chassisController;
    private GamepadEx armController;

    public void init() {
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);
    }

    public void loop() {
        chassisController.readButtons();
        if (chassisController.wasJustPressed(GamepadKeys.Button.X)) {
            robotBase.hangingMechanism.lower();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.Y)) {
            robotBase.hangingMechanism.raise();
        }
    }
}
