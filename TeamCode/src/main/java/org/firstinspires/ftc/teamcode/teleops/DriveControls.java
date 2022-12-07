package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.core.Poser;

@TeleOp(group = "drive")
@Config
public class DriveControls extends LinearOpMode {
  Poser poser = new Poser();
  BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

  BNO055IMU imu;
  DcMotorEx leftFront, leftRear, rightRear, rightFront, extender;
  Servo claw;

  boolean isGodModeEnabled = false;
  boolean godModeAlreadyToggled = false;
  Gamepad player1;
  Gamepad player2;

  // TODO: group these
  double gamepadMove;
  double gamepadAngle;
  double initialRobotAngle;
  double robotAngle;
  double gamepadTurnY;
  double movementAngle;
  double powerX;
  double powerY;

  public static double POSITION_SCALE = 0.75;
  public static double POSITION_DAMPENING = 4;
  public static double ROTATION_SCALE = 0.75;
  public static double ROTATION_DAMPENING = 4;
  public static double POSITION_REDUCTION = 0.5;

  public static double CLAW_CLOSED = 1;
  public static double CLAW_OPENED = 0.4;

  public static double EXTENDER_POWER = 0.5;
  public static int EXTENDER_MIN = 0;
  public static int EXTENDER_MAX = 3200;
  public static int EXTENDER_GROUND = EXTENDER_MIN;
  public static int EXTENDER_LOW = 1550;
  public static int EXTENDER_MEDIUM = 2850;
  public static int EXTENDER_HIGH = EXTENDER_MAX; // TODO: implement this
  public static int EXTENDER_SENSITIVITY = 20;
  int extenderOffset;
  int extenderState = EXTENDER_MIN;

  @Override
  public void runOpMode() throws InterruptedException {
    player1 = gamepad1;
    player2 = gamepad2;
    imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
    leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
    rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
    rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
    extender = hardwareMap.get(DcMotorEx.class, "extender");
    claw = hardwareMap.get(Servo.class, "claw");

    imu.initialize(imuParameters);

    leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    extenderOffset = extender.getCurrentPosition();
    extender.setTargetPositionTolerance(200);
    extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    extender.setTargetPosition(-extenderState + extenderOffset);
    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extender.setPower(EXTENDER_POWER);

    waitForStart();

    while (!isStopRequested()) {
      // ########## WHEELS ##########
      gamepadMove = poser.dampen(Math.hypot(-player1.left_stick_x, player1.left_stick_y), POSITION_DAMPENING, POSITION_SCALE) * (player1.right_bumper ? POSITION_REDUCTION : 1);
      gamepadAngle = Math.atan2(player1.left_stick_y, -player1.left_stick_x);
      gamepadTurnY = poser.dampen(player1.right_stick_x, ROTATION_DAMPENING, ROTATION_SCALE);
      robotAngle = imu.getAngularOrientation(
        AxesReference.INTRINSIC,
        AxesOrder.ZYX,
        AngleUnit.RADIANS
      ).firstAngle;
      movementAngle = -robotAngle - initialRobotAngle + gamepadAngle;
      powerX = gamepadMove * Math.sin(movementAngle);
      powerY = gamepadMove * Math.cos(movementAngle);

      leftFront.setPower(-powerY - powerX + gamepadTurnY);
      leftRear.setPower(powerY - powerX + gamepadTurnY);
      rightRear.setPower(-powerY - powerX - gamepadTurnY);
      rightFront.setPower(powerY - powerX - gamepadTurnY);

      if (player1.start) {
        if (player1.dpad_up){
          initialRobotAngle = 0;
        } else if (player1.dpad_right) {
          initialRobotAngle = -Math.PI / 2;
        } else if (player1.dpad_down) {
          initialRobotAngle = Math.PI;
        } else if (player1.dpad_left) {
          initialRobotAngle = Math.PI / 2;
        }
      }

      // ########## EXTENDER ##########
      if (!player2.start) {
        if (player2.a) extenderState = EXTENDER_GROUND;
        if (player2.x) extenderState = EXTENDER_LOW;
        if (player2.b) extenderState = EXTENDER_MEDIUM;
        if (player2.y) extenderState = EXTENDER_HIGH;
      }

      extenderState += EXTENDER_SENSITIVITY * player2.right_trigger;
      extenderState -= EXTENDER_SENSITIVITY * player2.left_trigger;
      extenderState = (int) Math.min(EXTENDER_MAX, Math.max(EXTENDER_MIN, extenderState));
      extender.setTargetPosition(-extenderState + extenderOffset);

      // ########## CLAW ##########
      claw.setPosition(player2.left_bumper ? CLAW_OPENED : CLAW_CLOSED);

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
      telemetry.addData("God Mode Already Toggled", godModeAlreadyToggled);
      telemetry.update();
    }
  }
}