package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Drive;
import org.firstinspires.ftc.teamcode.core.SleeveDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoRight", group = "Auto")
public class AutoRight extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);
    Drive drive = new Drive(hardwareMap, telemetry);

    TrajectorySequence t1 = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
      .forward(1)
      .strafeLeft(28)
      .build();

    TrajectorySequence t2 = mecanumDrive.trajectorySequenceBuilder(t1.end())
      .forward(36)
      .turn(Math.toRadians(-90))
      .forward(2.5)
      .build();

    TrajectorySequence t3 = mecanumDrive.trajectorySequenceBuilder(t2.end())
      .back(1)
      .strafeLeft(12)
      .build();

    TrajectorySequence t4 = mecanumDrive.trajectorySequenceBuilder(t3.end())
      .forward(46.5)
      .build();

    TrajectorySequence t5 = mecanumDrive.trajectorySequenceBuilder(t4.end())
      .back(46)
      .strafeRight(12)
      .forward(2)
      .build();

    TrajectorySequence t6 = mecanumDrive.trajectorySequenceBuilder(t5.end())
      .back(2)
      .build();

    TrajectorySequence t7 = mecanumDrive.trajectorySequenceBuilder(t6.end())
      .strafeRight(13)
      .forward(16)
      .build();

    TrajectorySequence t8First = mecanumDrive.trajectorySequenceBuilder(t7.end())
      .back(18)
      .build();

    TrajectorySequence t8Second = mecanumDrive.trajectorySequenceBuilder(t7.end())
      .forward(5)
      .build();

    TrajectorySequence t8Third = mecanumDrive.trajectorySequenceBuilder(t7.end())
      .forward(30)
      .build();

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(1000);
    drive.setExtenderLevel(Drive.ExtenderLevel.ABOVE_GROUND);

    auto.init();
    waitForStart();
    auto.readEnvironment();

    mecanumDrive.followTrajectorySequence(t1);
    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);

    mecanumDrive.followTrajectorySequence(t2);
    drive.addExtenderPosition(-750);
    sleep(1000);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(500);

    drive.addExtenderPosition(750);
    sleep(1000);

    mecanumDrive.followTrajectorySequence(t3);
    drive.setExtenderPosition(600);

    mecanumDrive.followTrajectorySequence(t4);
    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(500);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    sleep(500);

    mecanumDrive.followTrajectorySequence(t5);
    drive.addExtenderPosition(-750);
    sleep(500);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(500);

    drive.addExtenderPosition(750);
    sleep(750);

    mecanumDrive.followTrajectorySequence(t6);
    drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
    sleep(250);

    mecanumDrive.followTrajectorySequence(t7);
    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(500);

    drive.setExtenderLevel(Drive.ExtenderLevel.ABOVE_GROUND);

    switch (auto.side) {
      case NONE:
      case FIRST:
        mecanumDrive.followTrajectorySequence(t8First);
        break;
      case SECOND:
        mecanumDrive.followTrajectorySequence(t8Second);
        break;
      case THIRD:
        mecanumDrive.followTrajectorySequence(t8Third);
        break;
    }

    drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
    sleep(2000);

    auto.stop();
  }
}
