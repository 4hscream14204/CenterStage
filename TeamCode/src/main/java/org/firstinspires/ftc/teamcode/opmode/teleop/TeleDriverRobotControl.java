package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;

@TeleOp(name="DriverRobotControl")
public class TeleDriverRobotControl extends OpMode {

    public RobotBase robotBase;
    private GamepadEx chassisController;
    private GamepadEx armController;
    private double dblCurrentHeading = 0;
    private double dblChassisControllerLeftX = 0;
    private double dblChassisControllerLeftY = 0;
    private double dblChassisControllerRightX = 0;
    public void init() {
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);
    }
    public void loop() {
        chassisController.readButtons();
        dblChassisControllerLeftX = Math.abs(chassisController.getLeftX()) * chassisController.getLeftX();
        dblChassisControllerLeftY = Math.abs(chassisController.getLeftY()) * chassisController.getLeftY();
        dblChassisControllerRightX = Math.abs(chassisController.getRightX()) * chassisController.getRightX();
        dblCurrentHeading = robotBase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
            Vector2d input = new Vector2d(
                    dblChassisControllerLeftY,
                    -dblChassisControllerLeftX
            ).rotated(-dblCurrentHeading);

            robotBase.MecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -dblChassisControllerRightX
                    )
            );

        } else {

            robotBase.MecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            dblChassisControllerLeftY,
                            -dblChassisControllerLeftX,
                            -dblChassisControllerRightX
                    )
            );


        }

        if (chassisController.wasJustPressed(GamepadKeys.Button.X)) {
            robotBase.HangingMechanism.Lower();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.Y) && chassisController.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            robotBase.HangingMechanism.Raise();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.B) && chassisController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robotBase.AirplaneLauncher.Release();
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.START)) {
            if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
                robotBase.controlScheme = RobotBase.ChassisControlType.ROBOTCENTRIC;
            } else {
                robotBase.controlScheme = RobotBase.ChassisControlType.FIELDCENTRIC;
            }
        }

        robotBase.MecanumDrive.update();

        telemetry.addData("IMU yaw angle", robotBase.imu.getRobotYawPitchRollAngles());
    }
}
