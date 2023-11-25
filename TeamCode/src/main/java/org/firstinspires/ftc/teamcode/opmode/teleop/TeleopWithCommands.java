package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.GrabberSubsystem;

@TeleOp(name="TeleCommandDriverContol")
public class TeleopWithCommands extends OpMode {
    public RobotBase robotBase;
    private GamepadEx chassisController;
    private GamepadEx armController;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);

        armController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(() -> robotBase.grabber.toggleGrabber()));
        armController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> robotBase.grabber.toggleArm()));
        chassisController.getGamepadButton(GamepadKeys.Button.Y)
                .and(new GamepadButton(chassisController, GamepadKeys.Button.LEFT_BUMPER))
                .whenActive(new InstantCommand(() -> robotBase.hangingMechanism.raise()));
        armController.getGamepadButton(GamepadKeys.Button.A)
                        .whenHeld(new InstantCommand(() -> robotBase.odometryServos.odometryToggle()))
                .whenReleased(new InstantCommand(() -> robotBase.odometryServos.odometryStop()));
        chassisController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(new GamepadButton(chassisController, GamepadKeys.Button.X))
                .toggleWhenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> robotBase.airplaneLauncher.raiseAndLaunch()),
                        new WaitCommand(3000),
                        new InstantCommand(() -> robotBase.airplaneLauncher.raiseAndLaunch()),
                        new WaitCommand(6000),
                        new InstantCommand(() -> robotBase.airplaneLauncher.lower())));

    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}
