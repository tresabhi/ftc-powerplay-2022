package org.firstinspires.ftc.teamcode.autos;

import android.view.contentcapture.DataRemovalRequest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Drive;

@Autonomous(name = "CamTest", group = "Auto")
public class CamTest extends LinearOpMode {
  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    Drive drive = new Drive(hardwareMap, telemetry);

    auto.init(drive);
    waitForStart();

    while (!isStopRequested()) {
      auto.readSleeve();
      telemetry.addData("sleeve", auto.sleeveSide);
      telemetry.update();
    }
  }
}
