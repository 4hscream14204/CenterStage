package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name= "DPad Test")
public class DPadTest extends OpMode {

    private boolean bolDPadUp = false;
    private boolean bolDPadDown = false;
    private boolean bolDPadLeft = false;
    private boolean bolDPadRight = false;
    private GamepadEx dPadTestingController;
    public void init() {
        dPadTestingController = new GamepadEx(gamepad1);
    }
    public void loop() {

        dPadTestingController.readButtons();
        if(dPadTestingController.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            bolDPadUp = true;
        }
        if(dPadTestingController.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            bolDPadDown = true;
        }
        if(dPadTestingController.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            bolDPadLeft = true;
        }
        if(dPadTestingController.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            bolDPadRight = true;
        }


        telemetry.addData("DPad Up Pressed", bolDPadUp);
        telemetry.addData("DPad Down Pressed", bolDPadDown);
        telemetry.addData("DPad Left Pressed", bolDPadLeft);
        telemetry.addData("DPad Right Pressed", bolDPadRight);
    }
}
