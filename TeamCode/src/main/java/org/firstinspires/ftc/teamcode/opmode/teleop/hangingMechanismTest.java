package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;


@TeleOp(name="hangingMechanismTest")
public class hangingMechanismTest extends OpMode {

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
            robotBase.HangingMechanism.Lower();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.Y)) {
            robotBase.HangingMechanism.Raise();
        }
    }
}
