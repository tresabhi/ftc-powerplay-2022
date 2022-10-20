package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import kotlin.jvm.internal.Lambda;

public class Auto {
  HardwareMap hardwareMap;
  Telemetry telemetry;
  OpenCvCamera camera;
  SleeveDetector sleeveDetector;

  public Auto(HardwareMap hardwareMap, Telemetry telemetry) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;
  }

  public void init() {
    telemetry.addLine("autonomous API is initializing");

    telemetry.addLine("camera is initializing");

    int cameraMonitorViewId = hardwareMap.appContext.getResources()
      .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance()
      .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    SleeveDetector sleeveDetector = new SleeveDetector(telemetry);

    camera.setPipeline(sleeveDetector);
    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }

      @Override
      public void onError(int errorCode) {}
    });

    telemetry.addLine("camera initialized");

    telemetry.addLine("autonomous API initialized");
    telemetry.addLine("PRESS START");
  }

  public void stop() {
    camera.stopStreaming();
  }
}
