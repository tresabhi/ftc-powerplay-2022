package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Auto;

@Autonomous(name = "DistanceTest", group = "Auto")
public class DistanceTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);

    waitForStart();

    while (!isStopRequested()) {
      auto.readDistance();

      telemetry.addData("distance", auto.distance);
      telemetry.update();
    }
  }
}
