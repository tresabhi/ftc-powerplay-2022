package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Poser;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class RoadRunnerProgramTest extends LinearOpMode {
  @Override
  public void runOpMode() {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    Poser poser = new Poser();

    Pose2d initialPosition = poser.pose2d(48, -48, 0);

    drive.setPoseEstimate(initialPosition);
    drive.update();

    Trajectory right = drive.trajectoryBuilder(initialPosition)
      .lineToSplineHeading(poser.pose2d(48, 48, -90))
      .build();

    Trajectory top = drive.trajectoryBuilder(right.end())
      .lineToSplineHeading(poser.pose2d(-48, 48, -180))
      .build();

    Trajectory left = drive.trajectoryBuilder(top.end())
      .lineToSplineHeading(poser.pose2d(-48, -48, -270))
      .build();

    Trajectory bottom = drive.trajectoryBuilder(left.end())
      .lineToSplineHeading(poser.pose2d(48, -48, -360))
      .build();

    waitForStart();

    drive.followTrajectory(right);
    drive.followTrajectory(top);
    drive.followTrajectory(left);
    drive.followTrajectory(bottom);

    while (!isStopRequested()) {
      Pose2d poseEstimate = drive.getPoseEstimate();
      telemetry.addData("x", poseEstimate.getX());
      telemetry.addData("y", poseEstimate.getY());
      telemetry.addData("heading", poseEstimate.getHeading());
      telemetry.update();
    }
  }
}
