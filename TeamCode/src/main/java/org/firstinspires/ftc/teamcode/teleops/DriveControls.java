package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.core.Poser;

@TeleOp(group = "drive")
@Config
public class DriveControls extends LinearOpMode {
  Poser poser = new Poser();
  BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

  BNO055IMU imu;
  DcMotorEx leftFront, leftRear, rightRear, rightFront, extender;
  Servo claw;

  Orientation orientation;
  double gamepadMoveX;
  double gamepadMoveY;
  double gamepadMove;
  double gamepadAngle;
  double initialRobotAngle;
  double robotAngle;
  double gamepadTurnY;
  double movementAngle;
  double powerX;
  double powerY;

  public static double POSITION_SCALE = 0.75;
  public static double POSITION_DAMPENING = 2;
  public static double ROTATION_SCALE = 0.75;
  public static double ROTATION_DAMPENING = 2;

  public static double CLAW_CLOSED = 1;
  public static double CLAW_OPENED = 0.4;

  public static double EXTENDER_POWER = 0.5;
  public static int EXTENDER_MIN = 0;
  public static int EXTENDER_MAX = 3200;
  public static int EXTENDER_SENSITIVITY = 20;
  int extenderOffset;
  int extenderState = EXTENDER_MIN;

  @Override
  public void runOpMode() throws InterruptedException {
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
      gamepadMoveX = poser.dampen(-gamepad1.left_stick_x, POSITION_DAMPENING, POSITION_SCALE);
      gamepadMoveY = poser.dampen(gamepad1.left_stick_y, POSITION_DAMPENING, POSITION_SCALE);
      gamepadTurnY = poser.dampen(gamepad1.right_stick_x, ROTATION_DAMPENING, ROTATION_SCALE);
      gamepadMove = Math.hypot(gamepadMoveX, gamepadMoveY);
      gamepadAngle = Math.atan2(gamepadMoveY, gamepadMoveX);
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

      if (gamepad1.start) {
        if (gamepad1.dpad_up){
          initialRobotAngle = 0;
        } else if (gamepad1.dpad_right) {
          initialRobotAngle = -Math.PI / 2;
        } else if (gamepad1.dpad_down) {
          initialRobotAngle = Math.PI;
        } else if (gamepad1.dpad_left) {
          initialRobotAngle = Math.PI / 2;
        }
      }

      claw.setPosition(gamepad2.right_bumper ? CLAW_OPENED : CLAW_CLOSED);

      extenderState = (int) Math.min(
        EXTENDER_MAX,
        Math.max(
          EXTENDER_MIN,
          extenderState
            + EXTENDER_SENSITIVITY * gamepad2.right_trigger
            - EXTENDER_SENSITIVITY * gamepad2.left_trigger
        )
      );
      extender.setTargetPosition(-extenderState + extenderOffset);

      telemetry.addData("state", extenderState);
      telemetry.addData("calculated state", -extenderState);
      telemetry.addData("current pos", extender.getCurrentPosition());
      telemetry.addData("current target", extender.getTargetPosition());
      telemetry.update();
    }
  }
}