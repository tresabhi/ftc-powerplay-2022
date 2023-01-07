package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.Auto;

@Autonomous(name = "CamTest", group = "Auto")
public class CamTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);

    auto.init();
    waitForStart();
    auto.readEnvironment();

    while (!isStopRequested()) {

    }

    auto.cleanup();
  }
}
