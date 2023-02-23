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
    drive.setExtenderLevel(Drive.ExtenderLevel.STACK_5);
    auto.init(false);
    waitForStart();

    auto.approachConeStack(drive, 0, Auto.Alliance.Red);

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(999999);

    telemetry.update();
  }
}