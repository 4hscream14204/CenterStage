package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp (name= "DPad Test")
public class DPadTest extends OpMode {

    private boolean bolDPadUp = false;
    private boolean bolDPadDown = false;
    private boolean bolDPadLeft = false;
    private boolean bolDPadRight = false;
    private boolean bolRedButton = false;
    private boolean bolBlueButton = false;
    private boolean bolGreenButton = false;
    private boolean bolBlackButton = false;
    private TouchSensor tsRedButton;
    private TouchSensor tsBlueButton;
    private TouchSensor tsGreenButton;
    private TouchSensor tsBlackButton;

    private GamepadEx dPadTestingController;
    public void init() {
        dPadTestingController = new GamepadEx(gamepad1);
        tsRedButton = hardwareMap.get(TouchSensor.class,"RedButton");
        tsBlueButton = hardwareMap.get(TouchSensor.class,"BlueButton");
        tsGreenButton = hardwareMap.get(TouchSensor.class,"GreenButton");
        tsBlackButton = hardwareMap.get(TouchSensor.class,"BlackButton");
    }
    public void loop() {

        dPadTestingController.readButtons();
        if(dPadTestingController.getButton(GamepadKeys.Button.DPAD_UP)) {
            bolDPadUp = true;
        }
        if(dPadTestingController.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            bolDPadDown = true;
        }
        if(dPadTestingController.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            bolDPadLeft = true;
        }
        if(dPadTestingController.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            bolDPadRight = true;
        }
        if(tsRedButton.isPressed()) {
            bolRedButton = true;
        } else {
            bolRedButton = false;
        }
        if(tsBlueButton.isPressed()) {
            bolBlueButton = true;
        } else {
            bolBlueButton = false;
        }
        if(tsGreenButton.isPressed()) {
            bolGreenButton = true;
        } else {
            bolGreenButton = false;
        }
        if(tsBlackButton.isPressed()) {
            bolBlackButton = true;
        } else {
            bolBlackButton = false;
        }


        telemetry.addData("DPad Up Pressed", bolDPadUp);
        telemetry.addData("DPad Down Pressed", bolDPadDown);
        telemetry.addData("DPad Left Pressed", bolDPadLeft);
        telemetry.addData("DPad Right Pressed", bolDPadRight);
        telemetry.addData("Red Button Pressed", bolRedButton);
        telemetry.addData("Blue Button Pressed", bolBlueButton);
        telemetry.addData("Green Button Pressed", bolGreenButton);
        telemetry.addData("Black Button Pressed", bolBlackButton);
    }
}
