package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneLauncherSubsystem;

@TeleOp(name="DriverRobotControl")
public class TeleDriverRobotControl extends OpMode {


    public RobotBase robotBase;
    private GamepadEx chassisController;
    private GamepadEx armController;
    private double dblCurrentHeading = 0;
    private double dblChassisControllerLeftX = 0;
    private double dblChassisControllerLeftY = 0;
    private double dblChassisControllerRightX = 0;
    private String strLastButtonPressed = "";


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
        //dblCurrentHeading = robotBase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        dblCurrentHeading = angles.firstAngle;


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

        if (chassisController.wasJustPressed(GamepadKeys.Button.B)) {
            robotBase.HangingMechanism.Lower();
            strLastButtonPressed = "B";
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.Y) && chassisController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robotBase.HangingMechanism.Raise();
            strLastButtonPressed = "Y/LB";
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.X) && chassisController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robotBase.AirplaneLauncher.RaiseAndLaunch();
            strLastButtonPressed = "X/LB";
        } else if (chassisController.wasJustPressed(GamepadKeys.Button.X)) {
            robotBase.AirplaneLauncher.Lower();
            strLastButtonPressed = "X";
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.START)) {
            if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
                robotBase.controlScheme = RobotBase.ChassisControlType.ROBOTCENTRIC;
            } else {
                robotBase.controlScheme = RobotBase.ChassisControlType.FIELDCENTRIC;
            }
        }
            if (chassisController.isDown(GamepadKeys.Button.A)) {
                robotBase.OdometryServos.OdometryToggle();
            }
            if(chassisController.wasJustReleased(GamepadKeys.Button.A)) {
                robotBase.OdometryServos.OdometryStop();
            }
            robotBase.MecanumDrive.update();

            telemetry.addData("IMU yaw angle", robotBase.imu.getRobotYawPitchRollAngles());
            telemetry.addData("Chassis Control", robotBase.controlScheme);
        telemetry.addData("Button Pressed", strLastButtonPressed);
    }
}
