package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Auto {
  HardwareMap hardwareMap;
  Telemetry telemetry;
  OpenCvCamera camera;
  Drive drive;
  SleeveDetector sleeveDetector;
  ColorRangeSensor colorSensorLeft;
  RevColorSensorV3 colorSensorRight;
  public DistanceSensor distanceSensor;
  BNO055IMU imu;
  public SleeveDetector.Side sleeveSide;
  public double colorLeft = 0;
  public double colorRight = 0;
  public double distance = 0;
  double MIN_RUNTIME = 2000;
  double MAX_RUNTIME = 5000;
  double TARGET_DISTANCE = 47;

  public Auto(HardwareMap hardwareMap, Telemetry telemetry) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;
    this.sleeveDetector = new SleeveDetector(telemetry);
    this.colorSensorLeft = hardwareMap.get(ColorRangeSensor.class, "colorSensorLeft");
    this.colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");
    this.distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    this.imu = hardwareMap.get(BNO055IMU.class, "imu");
  }

  public void init(Drive drive) {
    init(drive, true);
  }

  public void init(Drive drive, boolean initCam) {
    this.drive = drive;

    telemetry.addLine("autonomous API is initializing");

    drive.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
    drive.rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    drive.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

    if (initCam) {
      telemetry.addLine("camera is initializing");

      int cameraMonitorViewId = hardwareMap.appContext
              .getResources()
              .getIdentifier(
                      "cameraMonitorViewId",
                      "id",
                      hardwareMap.appContext.getPackageName()
              );
      camera =
              OpenCvCameraFactory
                      .getInstance()
                      .createInternalCamera(
                              OpenCvInternalCamera.CameraDirection.BACK,
                              cameraMonitorViewId
                      );
      camera.setPipeline(sleeveDetector);
      camera.openCameraDeviceAsync(
              new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                  camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
                }

                @Override
                public void onError(int errorCode) {
                  telemetry.addLine("ERROR: pipeline failed #" + errorCode);
                }
              }
      );

      telemetry.addLine("camera initialized");
    }

    BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
    imuParameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    imu.initialize(imuParameters);

    telemetry.addLine("autonomous API initialized");
    telemetry.addLine("PRESS START");
  }

  public void readSleeve() {
    sleeveSide = sleeveDetector.getSide();

    stopCam();

    telemetry.clearAll();
    telemetry.addLine("side detected " + sleeveSide.name());
    telemetry.update();
  }

  public void stopCam() {
    camera.stopStreaming();
  }

  public void approachConeStack(double targetAngle) throws InterruptedException {
    double offsetPower = 1, rotationOffsetPower = 1, distancePower = 1;
    double time = 0;
    double startTime = System.currentTimeMillis();

    while ((distance <= TARGET_DISTANCE || (time) < MIN_RUNTIME) && (time) < MAX_RUNTIME) {
      time = System.currentTimeMillis() - startTime;

      colorLeft = colorSensorLeft.getRawLightDetected() / 2;
      colorRight = colorSensorRight.getRawLightDetected();
      distance = distanceSensor.getDistance(DistanceUnit.MM);

      double offsetRaw = colorRight - colorLeft;
      double offsetScaled = Math.max(-1, Math.min(1, offsetRaw / 50));
      offsetPower = offsetScaled * 0.11;

      double robotAngle = imu.getAngularOrientation(
              AxesReference.INTRINSIC,
              AxesOrder.ZYX,
              AngleUnit.RADIANS
      ).firstAngle % (2 * Math.PI);
      double rotationOffsetRaw = targetAngle - robotAngle;
      double rotationOffsetScaled = Math.max(-1, Math.min(1, rotationOffsetRaw / Math.toRadians(6)));
      rotationOffsetPower = rotationOffsetScaled * 0.18;

      double distanceDiffRaw = distance - TARGET_DISTANCE;
      double distanceDiff = Math.max(-1, Math.min(1, (distanceDiffRaw) / 25));
      distancePower = distanceDiff * 0.12;

      drive.leftFront.setPower(offsetPower - rotationOffsetPower + distancePower);
      drive.leftRear.setPower(-offsetPower - rotationOffsetPower + distancePower);
      drive.rightRear.setPower(offsetPower + rotationOffsetPower + distancePower);
      drive.rightFront.setPower(-offsetPower + rotationOffsetPower + distancePower);

      telemetry.addData("distance", distance);
      telemetry.addData("l", colorLeft);
      telemetry.addData("r", colorRight);
      telemetry.update();
    }

    drive.leftFront.setPower(0);
    drive.leftRear.setPower(0);
    drive.rightRear.setPower(0);
    drive.rightFront.setPower(0);

    if (time >= MAX_RUNTIME) {
      throw new InterruptedException();
    }
  }
}
