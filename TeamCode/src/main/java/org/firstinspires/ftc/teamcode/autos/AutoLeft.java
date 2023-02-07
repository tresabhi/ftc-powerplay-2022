package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Drive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "AutoLeft", group = "Auto")
public class AutoLeft extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    SampleMecanumDrive mecanum = new SampleMecanumDrive(hardwareMap);
    Drive drive = new Drive(hardwareMap, telemetry);

    TrajectorySequence t1 = mecanum
            .trajectorySequenceBuilder(new Pose2d())
            .strafeRight(20.5)
            .build();

    TrajectorySequence t2 = mecanum
            .trajectorySequenceBuilder(t1.end())
            .forward(24)
            .lineToLinearHeadingRelative(new Pose2d(-2.25, 15, Math.toRadians(90)))
            .build();

    TrajectorySequence t3 = mecanum
            .trajectorySequenceBuilder(t2.end())
            .strafeRight(13.5)
            .build();

    TrajectorySequence t4 = mecanum
            .trajectorySequenceBuilder(t3.end())
            .forward(46.75)
            .back(0.75)
            .build();

    TrajectorySequence t5 = mecanum
            .trajectorySequenceBuilder(t4.end())
            .lineToLinearHeadingRelative(new Pose2d(-1, -38.75, Math.toRadians(90)))
            .forward(2.25)
            .build();

    TrajectorySequence t6 = mecanum
            .trajectorySequenceBuilder(t5.end())
            .back(3.25)
            .lineToLinearHeadingRelative(new Pose2d(12, 0, Math.toRadians(-90)))
            .lineToLinearHeadingRelative(new Pose2d(0.5, 26))
            .back(0.75)
            .build();

    TrajectorySequence t7 = mecanum
            .trajectorySequenceBuilder(t6.end())
            .lineToLinearHeadingRelative(new Pose2d(-2.75, -37, Math.toRadians(90)))
            .forward(2)
            .build();

    TrajectorySequence t8 = mecanum
            .trajectorySequenceBuilder(t7.end())
            .back(3)
            .build();

    TrajectorySequence t9_1 = mecanum
            .trajectorySequenceBuilder(t8.end())
            .strafeRight(36)
            .build();

    TrajectorySequence t9_2 = mecanum
            .trajectorySequenceBuilder(t8.end())
            .strafeRight(12)
            .build();

    TrajectorySequence t9_3 = mecanum
            .trajectorySequenceBuilder(t8.end())
            .strafeLeft(12)
            .build();

    drive.setClawState(Drive.ClawState.CLOSE);

    auto.init();
    waitForStart();
    auto.readEnvironment();

    drive.setExtenderLevel(Drive.ExtenderLevel.ABOVE_GROUND);

    mecanum.followTrajectorySequence(t1);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);

    mecanum.followTrajectorySequence(t2);
    sleep(500);

    drive.addExtenderPosition(-200);
    sleep(250);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(250);

    drive.addExtenderPosition(200);
    sleep(250);

    mecanum.followTrajectorySequence(t3);

    drive.setExtenderLevel(Drive.ExtenderLevel.STACK_5);
    mecanum.followTrajectorySequence(t4);

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(250);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    sleep(250);

    mecanum.followTrajectorySequence(t5);
    sleep(500);

    drive.addExtenderPosition(-200);
    sleep(250);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(250);

    drive.addExtenderPosition(200);
    sleep(250);

    drive.setExtenderLevel(Drive.ExtenderLevel.STACK_4);
    mecanum.followTrajectorySequence(t6);

    drive.setClawState(Drive.ClawState.CLOSE);
    sleep(250);

    drive.setExtenderLevel(Drive.ExtenderLevel.MEDIUM);
    sleep(250);

    mecanum.followTrajectorySequence(t7);
    sleep(500);

    drive.addExtenderPosition(-200);
    sleep(250);

    drive.setClawState(Drive.ClawState.OPEN);
    sleep(250);

    drive.addExtenderPosition(200);
    sleep(250);

    mecanum.followTrajectorySequence(t8);

    drive.setExtenderLevel(Drive.ExtenderLevel.GROUND);
    switch (auto.side) {
      case NONE:
      case FIRST:
        mecanum.followTrajectorySequence(t9_1);
        break;
      case SECOND:
        mecanum.followTrajectorySequence(t9_2);
        break;
      case THIRD:
        mecanum.followTrajectorySequence(t9_3);
        break;
    }

    sleep(2000);
    auto.cleanup();
  }
}
