package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Auto {
  HardwareMap hardwareMap;
  Telemetry telemetry;
  OpenCvCamera camera;
  SleeveDetector sleeveDetector;
  public SleeveDetector.Side sleeveSide;
  public double distance = -1;

  public Auto(HardwareMap hardwareMap, Telemetry telemetry) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;
    this.sleeveDetector = new SleeveDetector(telemetry);
  }

  public void init() {
    telemetry.addLine("autonomous API is initializing");
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

    telemetry.addLine("autonomous API initialized");
    telemetry.addLine("PRESS START");
  }

  public void readSleeve() {
    sleeveSide = sleeveDetector.getSide();

    camera.stopStreaming();

    telemetry.clearAll();
    telemetry.addLine("side detected " + sleeveSide.name());
    telemetry.update();
  }

  public void readDistance() {
//    distance = distanceSensor.;
  }
}
