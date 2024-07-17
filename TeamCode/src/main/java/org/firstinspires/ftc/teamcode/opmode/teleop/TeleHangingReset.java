package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;

@TeleOp(name="Hanging Reset")
public class TeleHangingReset extends OpMode {

    public RobotBase robotBase;
    private GamepadEx resetController;

    public void init() {

        CommandScheduler.getInstance().reset();
        robotBase = new RobotBase(hardwareMap);
        resetController = new GamepadEx(gamepad1);

        //HANGING RESET
        resetController.getGamepadButton(GamepadKeys.Button.Y)
                .whileHeld(()->CommandScheduler.getInstance().schedule(
                        new ConditionalCommand(
                                new InstantCommand(()->robotBase.hangingMechanismSubsystem.stop()),
                                new InstantCommand(()->robotBase.hangingMechanismSubsystem.moveDown()),
                                ()-> robotBase.hangingSensorSubsystem.hangingIsDown()
                        )
                ))
                .whenReleased(()->CommandScheduler.getInstance().schedule(
                        new InstantCommand(()->robotBase.hangingMechanismSubsystem.stop())
                        )
                );

    }

    public void loop() {

        resetController.readButtons();

        CommandScheduler.getInstance().run();

    }
}
