package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Auto {
  HardwareMap hardwareMap;
  Telemetry telemetry;
  OpenCvCamera camera;
  SleeveDetector sleeveDetector;
  ColorRangeSensor colorSensorLeft;
  RevColorSensorV3 colorSensorRight;
  BNO055IMU imu;
  public SleeveDetector.Side sleeveSide;
  public double colorLeft = 0;
  public double colorRight = 0;

  public Auto(HardwareMap hardwareMap, Telemetry telemetry) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;
    this.sleeveDetector = new SleeveDetector(telemetry);
    this.colorSensorLeft = hardwareMap.get(ColorRangeSensor.class, "colorSensorLeft");
    this.colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");
    this.imu = hardwareMap.get(BNO055IMU.class, "imu");

  }

  public void init() {
    init(true);
  }

  public void init(boolean initCam) {
    telemetry.addLine("autonomous API is initializing");

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

  public void readColor() {
    colorLeft = colorSensorLeft.getRawLightDetected();
    colorRight = colorSensorRight.getRawLightDetected();
  }

  public void approachConeStack(Drive drive, double targetAngle) {
    readColor();

    double offsetRaw = colorRight - colorLeft;
    double offsetScaled = Math.max(-1, Math.min(1, offsetRaw / 150)) ;
    double offsetPower = offsetScaled * 0.25;
    double robotAngle = imu.getAngularOrientation(
            AxesReference.INTRINSIC,
            AxesOrder.ZYX,
            AngleUnit.RADIANS
    ).firstAngle % (2 * Math.PI);
    double rotationOffsetRaw = targetAngle - robotAngle;
    double rotationOffsetScaled = Math.max(-1, Math.min(1, rotationOffsetRaw / Math.toRadians(4)));
    double rotationOffsetPower = rotationOffsetScaled * 0.25;

    drive.leftFront.setPower(offsetPower - rotationOffsetPower);
    drive.leftRear.setPower(-offsetPower - rotationOffsetPower);
    drive.rightRear.setPower(offsetPower + rotationOffsetPower);
    drive.rightFront.setPower(-offsetPower + rotationOffsetPower);
  }
}
