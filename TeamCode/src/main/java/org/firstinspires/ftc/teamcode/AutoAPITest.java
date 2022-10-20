package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.Auto;

@Autonomous(name = "AutoAPITest", group = "Auto")
public class AutoAPITest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);

    auto.init();
    waitForStart();

    while (!isStopRequested()) {
      // ...
    }

    auto.stop();
  }
}
