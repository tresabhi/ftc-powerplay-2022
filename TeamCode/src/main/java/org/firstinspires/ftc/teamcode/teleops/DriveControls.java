package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.core.Drive;
import org.firstinspires.ftc.teamcode.core.Poser;

@TeleOp(group = "drive")
@Config
public class DriveControls extends LinearOpMode {

  Poser poser = new Poser();
  Drive drive;

  BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

  BNO055IMU imu;
  DcMotorEx leftFront, leftRear, rightRear, rightFront;

  boolean isGodModeEnabled = false;
  boolean godModeAlreadyToggled = false;
  Gamepad player1;
  Gamepad player2;

  double initialRobotAngle;

  public static double SPEED_NORMAL = 0.45;
  public static double SPEED_LOW = 0.2;
  public static double SPEED_DAMP = 4;
  public static double ROTATION_NORMAL = 0.5;
  public static double ROTATION_SLOW = 0.3;
  public static double ROTATION_DAMPENING = 4;

  public static int EXTENDER_SENSITIVITY = 20;

  @Override
  public void runOpMode() throws InterruptedException {
    drive = new Drive(hardwareMap, telemetry);

    player1 = gamepad1;
    player2 = gamepad2;
    imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
    rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

    imu.initialize(imuParameters);

    leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // make it faster in driver controls
    drive.extender.setPower(1);

    waitForStart();

    while (!isStopRequested()) {
      // ########## WHEELS ##########
      double gamepadMove =
        poser.dampen(
          Math.hypot(-player1.left_stick_x, player1.left_stick_y),
          SPEED_DAMP,
          player1.right_bumper ? SPEED_LOW : SPEED_NORMAL
        );
      double squareMagnitude =
        1 /
        (
          Math.cos(
            (
              (
                Math.atan2(Math.abs(player1.left_stick_y), player1.left_stick_x) +
                (Math.PI / 4)
              ) %
              (Math.PI / 2)
            ) -
            (Math.PI / 4)
          )
        );
      double gamepadAngle = Math.atan2(
        player1.left_stick_y,
        -player1.left_stick_x
      );
      double gamepadTurnY = poser.dampen(
        player1.right_stick_x,
        ROTATION_DAMPENING,
        player1.right_bumper ? ROTATION_SLOW : ROTATION_NORMAL
      );
      double robotAngle = imu.getAngularOrientation(
        AxesReference.INTRINSIC,
        AxesOrder.ZYX,
        AngleUnit.RADIANS
      ).firstAngle;
      double movementAngle = -robotAngle - initialRobotAngle + gamepadAngle;
      double powerX = gamepadMove * Math.sin(movementAngle) / squareMagnitude;
      double powerY = gamepadMove * Math.cos(movementAngle) / squareMagnitude;

      leftFront.setPower(-powerY - powerX + gamepadTurnY);
      leftRear.setPower(powerY - powerX + gamepadTurnY);
      rightRear.setPower(-powerY - powerX - gamepadTurnY);
      rightFront.setPower(powerY - powerX - gamepadTurnY);

      if (player1.start) {
        if (player1.dpad_up) {
          initialRobotAngle = 0;
        } else if (player1.dpad_right) {
          initialRobotAngle = -Math.PI / 2;
        } else if (player1.dpad_down) {
          initialRobotAngle = Math.PI;
        } else if (player1.dpad_left) {
          initialRobotAngle = Math.PI / 2;
        }
      }

      if (player1.right_stick_button) imu.initialize(imuParameters);

      // ########## EXTENDER ##########
      if (!player2.start) {
        if (player2.a) drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
        if (player2.x) drive.setExtenderLevel(Drive.ExtenderLevel.LOW);
        if (player2.b) drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
        if (player2.y) drive.setExtenderLevel(Drive.ExtenderLevel.HIGH);
      }

      drive.addExtenderPosition(EXTENDER_SENSITIVITY * player2.right_trigger);
      drive.addExtenderPosition(-EXTENDER_SENSITIVITY * player2.left_trigger);

      // ########## CLAW ##########
      drive.setClawState(
        player2.left_bumper ? Drive.ClawState.OPEN : Drive.ClawState.CLOSE
      );

      // ########## GOD MODE ##########
      if (gamepad1.start && gamepad1.back) {
        if (!godModeAlreadyToggled) {
          if (isGodModeEnabled) {
            player1 = gamepad1;
            player2 = gamepad2;
            isGodModeEnabled = false;
          } else {
            player1 = gamepad1;
            player2 = gamepad1;
            isGodModeEnabled = true;
          }

          godModeAlreadyToggled = true;
        }
      } else {
        godModeAlreadyToggled = false;
      }

      if (gamepad2.start && gamepad2.back) {
        if (!godModeAlreadyToggled) {
          if (isGodModeEnabled) {
            player1 = gamepad1;
            player2 = gamepad2;
            isGodModeEnabled = false;
          } else {
            player1 = gamepad2;
            player2 = gamepad2;
            isGodModeEnabled = true;
          }

          godModeAlreadyToggled = true;
        }
      } else {
        godModeAlreadyToggled = false;
      }

      // ########## TELEMETRY ##########
      telemetry.addData("God Mode", isGodModeEnabled);
      telemetry.addData("m", squareMagnitude);
      telemetry.addData("t", Math.atan2(player1.left_stick_y, player1.left_stick_x) / Math.PI * 180);
      telemetry.update();
    }
  }
}
