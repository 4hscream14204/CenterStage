package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.RobotBase;
import org.firstinspires.ftc.teamcode.subsystems.DataStorageSubsystem;

@TeleOp(name="DriverRobotControl")
public class TeleDriverRobotControl extends OpMode {


    public RobotBase robotBase;
    private GamepadEx chassisController;
    private GamepadEx armController;
    private double dblCurrentHeading = 0;
    private double dblChassisControllerRightX = 0;
    private double dblChassisControllerRightY = 0;
    private double dblChassisControllerLeftX = 0;
    private String strLastButtonPressed = "";


    public void init() {
        robotBase = new RobotBase(hardwareMap);
        chassisController = new GamepadEx(gamepad1);
        armController = new GamepadEx(gamepad2);
    }

    public void loop() {
        chassisController.readButtons();
        armController.readButtons();
        dblChassisControllerRightX = Math.abs(chassisController.getRightX()) * chassisController.getRightX();
        dblChassisControllerRightY = Math.abs(chassisController.getRightY()) * chassisController.getRightY();
        dblChassisControllerLeftX = Math.abs(chassisController.getLeftX()) * chassisController.getLeftX();
        //dblCurrentHeading = robotBase.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        Orientation angles = robotBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        dblCurrentHeading = angles.firstAngle + DataStorageSubsystem.dblIMUFinalHeading;


        if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
            Vector2d input = new Vector2d(
                    -dblChassisControllerRightY,
                    -dblChassisControllerRightX
            ).rotated(-dblCurrentHeading);

            robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -dblChassisControllerLeftX
                    )
            );

        } else {

            robotBase.mecanumDriveSubsystem.setWeightedDrivePower(
                    new Pose2d(
                            -dblChassisControllerRightY,
                            -dblChassisControllerRightX,
                            -dblChassisControllerLeftX
                    )
            );


        }

        if (chassisController.wasJustPressed(GamepadKeys.Button.B)) {
            robotBase.hangingMechanismSubsystem.lower();
            strLastButtonPressed = "B";
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.Y) && chassisController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robotBase.hangingMechanismSubsystem.raise();
            strLastButtonPressed = "Y/LB";
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.X) && chassisController.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robotBase.airplaneLauncherSubsystem.raiseAndLaunch();
            strLastButtonPressed = "X/LB";
        } else if (chassisController.wasJustPressed(GamepadKeys.Button.X)) {
            robotBase.airplaneLauncherSubsystem.lower();
            strLastButtonPressed = "X";
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.START)) {
            if (robotBase.controlScheme == RobotBase.ChassisControlType.FIELDCENTRIC) {
                robotBase.controlScheme = RobotBase.ChassisControlType.ROBOTCENTRIC;
            } else {
                robotBase.controlScheme = RobotBase.ChassisControlType.FIELDCENTRIC;
            }
        }
        if (chassisController.wasJustPressed(GamepadKeys.Button.BACK)) {
            DataStorageSubsystem.dblIMUFinalHeading = 0;
            robotBase.navxMicro.initialize();
        }
            if (armController.isDown(GamepadKeys.Button.A)) {
                robotBase.odometryServosSubsystem.odometryToggle();
            }
            if(armController.wasJustReleased(GamepadKeys.Button.A)) {
                robotBase.odometryServosSubsystem.odometryStop();
            }
            if(armController.wasJustPressed(GamepadKeys.Button.X)) {
                robotBase.grabber.toggleGrabber();
            }
            if(armController.wasJustPressed(GamepadKeys.Button.B)) {
                robotBase.grabber.toggleArm();
            }
            robotBase.mecanumDriveSubsystem.update();

            telemetry.addData("IMU yaw angle", robotBase.imu.getRobotYawPitchRollAngles());
            telemetry.addData("Chassis Control", robotBase.controlScheme);
            telemetry.addData("Button Pressed", strLastButtonPressed);
    }
}
