package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
  DcMotorEx leftFront, leftRear, rightRear, rightFront;

  Orientation orientation;
  double gamepadMoveX;
  double gamepadMoveY;
  double gamepadMove;
  double gamepadAngle;
  double initialRobotAngle; // TODO: set via dpad controls
  double robotAngle;
  double gamepadTurnY;
  double movementAngle;
  double powerX;
  double powerY;

  public static double POSITION_SCALE = 0.75;
  public static double POSITION_DAMPENING = 2;
  public static double ROTATION_SCALE = 0.75;
  public static double ROTATION_DAMPENING = 2;

  @Override
  public void runOpMode() throws InterruptedException {
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
    }
  }
}