package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Drive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "![NEW]AutoRight", group = "Auto")
public class AutoRightNew extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    SampleMecanumDrive mecanum = new SampleMecanumDrive(hardwareMap);
    Drive drive = new Drive(hardwareMap, telemetry);

    TrajectorySequence t1 = mecanum
      .trajectorySequenceBuilder(new Pose2d())
      .lineToLinearHeadingRelative(new Pose2d(-26.5, 1))
      .build();

    TrajectorySequence t2 = mecanum
      .trajectorySequenceBuilder(t1.end())
      .forward(37.25)
      .turn(Math.toRadians(90))
      .forward(2.5)
      .build();

    TrajectorySequence t3 = mecanum
      .trajectorySequenceBuilder(t2.end())
      .back(3)
      .lineToLinearHeadingRelative(new Pose2d(11.75, 0, Math.toRadians(180)))
      .forward(48.5)
      .build();

    TrajectorySequence t4 = mecanum
      .trajectorySequenceBuilder(t3.end())
      .back(35.5)
      .turn(Math.toRadians(-90))
      .forward(2.25)
      .build();

    TrajectorySequence t5 = mecanum
      .trajectorySequenceBuilder(t4.end())
      .back(1.75)
      .turn(Math.toRadians(90))
      .lineToLinearHeadingRelative(new Pose2d(0.5, 35))
      .build();

    TrajectorySequence t6 = mecanum
      .trajectorySequenceBuilder(t5.end())
      .back(11)
      .turn(Math.toRadians(-90))
      .forward(2.5)
      .build();

    TrajectorySequence t7 = mecanum
      .trajectorySequenceBuilder(t6.end())
      .back(2.5)
      .build();

    TrajectorySequence t8_1 = mecanum
      .trajectorySequenceBuilder(t7.end())
      .strafeLeft(11)
      .build();

    TrajectorySequence t8_2 = mecanum
      .trajectorySequenceBuilder(t7.end())
      .strafeRight(11)
      .build();

    TrajectorySequence t8_3 = mecanum
      .trajectorySequenceBuilder(t7.end())
      .strafeRight(35)
      .build();

    drive.setClawState(Drive.ClawState.CLOSE);

    auto.init();
    waitForStart();
    auto.readEnvironment();

    drive.setExtenderLevel(Drive.ExtenderLevel.ABOVE_GROUND);

    mecanum.followTrajectorySequence(t1);

    drive.setExtenderLevel(Drive.ExtenderLevel.HIGH);
    mecanum.followTrajectorySequence(t2);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    sleep(1000);

    drive.setClawState(Drive.ClawState.OPEN);
    drive.setExtenderLevel(Drive.ExtenderLevel.STACK_5);
    mecanum.followTrajectorySequence(t3);

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(250);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    mecanum.followTrajectorySequence(t4);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(250);

    drive.setExtenderLevel(Drive.ExtenderLevel.STACK_4);
    mecanum.followTrajectorySequence(t5);

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(500);

    drive.setExtenderLevel(Drive.ExtenderLevel.LOW);
    mecanum.followTrajectorySequence(t6);

    drive.setClawState(Drive.ClawState.OPEN);
    mecanum.followTrajectorySequence(t7);

    drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
    switch (auto.side) {
      case NONE:
      case FIRST:
        mecanum.followTrajectorySequence(t8_1);
        break;
      case SECOND:
        mecanum.followTrajectorySequence(t8_2);
        break;
      case THIRD:
        mecanum.followTrajectorySequence(t8_3);
        break;
    }

    sleep(2000);

    auto.cleanup();
  }
}
