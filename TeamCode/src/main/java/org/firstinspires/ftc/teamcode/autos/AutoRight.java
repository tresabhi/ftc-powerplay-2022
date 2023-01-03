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
    SampleMecanumDrive mecanum = new SampleMecanumDrive(hardwareMap);
    Drive drive = new Drive(hardwareMap, telemetry);

    TrajectorySequence t1 = mecanum
      .trajectorySequenceBuilder(new Pose2d())
      .forward(1)
      .strafeLeft(27.5)
      .build();

    TrajectorySequence t2 = mecanum
      .trajectorySequenceBuilder(t1.end())
      .forward(36.5)
      .turn(Math.toRadians(-90))
      .forward(2)
      .build();

    TrajectorySequence t3 = mecanum
      .trajectorySequenceBuilder(t2.end())
      .back(1.5)
      .strafeLeft(11)
      .build();

    TrajectorySequence t4 = mecanum
      .trajectorySequenceBuilder(t3.end())
      .forward(46.5)
      .build();

    TrajectorySequence t5 = mecanum
      .trajectorySequenceBuilder(t4.end())
      .back(46)
      .strafeRight(12)
      .forward(2)
      .build();

    TrajectorySequence t6 = mecanum
      .trajectorySequenceBuilder(t5.end())
      .back(2)
      .build();

    TrajectorySequence t7 = mecanum
      .trajectorySequenceBuilder(t6.end())
      .strafeLeft(12)
      .build();

    TrajectorySequence t8Second = mecanum
      .trajectorySequenceBuilder(t7.end())
      .forward(22.5)
      .build();

    TrajectorySequence t8Third = mecanum
      .trajectorySequenceBuilder(t7.end())
      .forward(46)
      .build();

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(1250);

    auto.init();
    waitForStart();
    auto.readEnvironment();

    drive.setExtenderLevel(Drive.ExtenderLevel.ABOVE_GROUND);
    sleep(500);

    mecanum.followTrajectorySequence(t1);
    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);

    mecanum.followTrajectorySequence(t2);
    drive.addExtenderPosition(-750);
    sleep(1000);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(500);

    drive.addExtenderPosition(750);
    sleep(1000);

    mecanum.followTrajectorySequence(t3);
    drive.setExtenderPosition(600);

    mecanum.followTrajectorySequence(t4);
    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(500);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    sleep(500);

    mecanum.followTrajectorySequence(t5);
    drive.addExtenderPosition(-750);
    sleep(500);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(500);

    drive.addExtenderPosition(750);
    sleep(750);

    mecanum.followTrajectorySequence(t6);
    drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
    sleep(250);

    mecanum.followTrajectorySequence(t7);

    switch (auto.side) {
      case NONE:
      case SECOND:
        mecanum.followTrajectorySequence(t8Second);
        break;
      case THIRD:
        mecanum.followTrajectorySequence(t8Third);
        break;
    }

    //    drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
    sleep(2000);

    auto.stop();
  }
}
