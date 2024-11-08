package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoWiringTest")
public class TeleServoWiringTest extends OpMode {

    private GamepadEx testingController;
    private Servo srvServo1;
    private Servo srvServo2;
    private Servo srvServo3;
    private Servo srvServo4;
    private Servo srvServo5;
    private Servo srvServo6;

    public void init() {
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
