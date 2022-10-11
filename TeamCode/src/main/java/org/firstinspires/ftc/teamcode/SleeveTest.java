package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.SleeveDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "AA-SleeveTest", group = "Auto")
public class SleeveTest extends LinearOpMode {
  OpenCvCamera camera;

  @Override
  public void runOpMode() throws InterruptedException {
    int cameraMonitorViewId = hardwareMap.appContext.getResources()
      .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance()
      .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    SleeveDetector sleeveDetector = new SleeveDetector(telemetry);

    camera.setPipeline(sleeveDetector);
    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
      }

      @Override
      public void onError(int errorCode) {}
    });


    waitForStart();

    while (!isStopRequested()) {
      switch (sleeveDetector.getSide()) {
        case NONE:
        case FIRST: {

        }
        case SECOND: {

        }
        case THIRD: {

        }
      }
    }

    camera.stopStreaming();
  }
}
