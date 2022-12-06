package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.SleeveDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "CamTest", group = "Auto")
public class CamTest extends LinearOpMode {
  SleeveDetector.Side side;
  OpenCvCamera camera;

  public void runOpMode() throws InterruptedException {
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    SleeveDetector sleeveDetector = new SleeveDetector(telemetry);
    camera.setPipeline(sleeveDetector);
    camera.openCameraDeviceAsync(
      new OpenCvCamera.AsyncCameraOpenListener() {
        @Override
        public void onOpened() {
          camera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
        }

        @Override
        public void onError(int i) {
          telemetry.addLine("Camera error "+ i);
        }
      }
    );

    waitForStart();

    while (!isStopRequested()) {
      telemetry.addData("side", sleeveDetector.getSide());
      telemetry.update();
    }

    camera.stopStreaming();
  }
}
