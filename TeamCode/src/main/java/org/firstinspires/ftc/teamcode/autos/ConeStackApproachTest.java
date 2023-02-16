package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Drive;

@Autonomous(name = "![AAPROTO]ConeStackApproachTest", group = "Auto")
public class ConeStackApproachTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    Drive drive = new Drive(hardwareMap, telemetry);

    auto.init(false);
    waitForStart();

    while (!isStopRequested()) {
      auto.approachConeStack(drive, 0);
    }
  }
}
