package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import kotlin.jvm.internal.Lambda;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SleeveDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class Auto {
  HardwareMap hardwareMap;
  Telemetry telemetry;
  OpenCvCamera camera;
  SleeveDetector sleeveDetector;
  SleeveDetector.Side side;

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
          camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
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

  public void stop() {
    camera.stopStreaming();
  }

  public void readEnvironment() {
    side = sleeveDetector.getSide();
    telemetry.addLine("side detected" + side.name());
  }
}
