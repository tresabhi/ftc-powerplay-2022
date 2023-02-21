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

    drive.setClawState(Drive.ClawState.OPEN);
    auto.init(false);
    waitForStart();

    drive.setExtenderLevel(Drive.ExtenderLevel.STACK_5);
    sleep(500);

    auto.approachConeStack(drive, 0, Auto.Alliance.Blue);

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(2000);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    sleep(99999);

    telemetry.update();
  }
}
