package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoWiringTest")
public class TeleServoWiringTest extends OpMode {

    private GamepadEx testingController;
    Servo srvServo1;
    Servo srvServo2;
    Servo srvServo3;
    Servo srvServo4;
    Servo srvServo5;
    Servo srvServo6;

    public void init() {

        srvServo1 = hardwareMap.get(Servo.class, "servo1");
        srvServo2 = hardwareMap.get(Servo.class, "servo2");
        srvServo3 = hardwareMap.get(Servo.class, "servo3");
        srvServo4 = hardwareMap.get(Servo.class, "servo4");
        srvServo5 = hardwareMap.get(Servo.class, "servo5");
        srvServo6 = hardwareMap.get(Servo.class, "servo6");

        CommandScheduler.getInstance().reset();
        testingController = new GamepadEx(gamepad1);

        testingController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo1.setPosition(1))
                ))
                .whenReleased(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo1.setPosition(0))
                ));

        testingController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo2.setPosition(1))
                ))
                .whenReleased(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo2.setPosition(0))
                ));

        testingController.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo3.setPosition(1))
                ))
                .whenReleased(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo3.setPosition(0))
                ));

        testingController.getGamepadButton(GamepadKeys.Button.X)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo4.setPosition(1))
                ))
                .whenReleased(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo4.setPosition(0))
                ));

        testingController.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo5.setPosition(1))
                ))
                .whenReleased(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo5.setPosition(0))
                ));

        testingController.getGamepadButton(GamepadKeys.Button.A)
                .whileHeld(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo6.setPosition(1))
                ))
                .whenReleased(()-> CommandScheduler.getInstance().schedule(
                        new InstantCommand(()-> srvServo6.setPosition(0))
                ));

    }
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
