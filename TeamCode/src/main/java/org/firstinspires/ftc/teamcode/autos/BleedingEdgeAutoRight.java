package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Drive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "![BleedingEdge]AutoRight", group = "Auto")
public class BleedingEdgeAutoRight extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    SampleMecanumDrive mecanum = new SampleMecanumDrive(hardwareMap);
    Drive drive = new Drive(hardwareMap, telemetry);

    TrajectorySequence t1 = mecanum
      .trajectorySequenceBuilder(new Pose2d())
      .lineToLinearHeadingRelative(new Pose2d(10, -10, Math.toRadians(90)))
      .lineToLinearHeadingRelative(new Pose2d(-10, -10))
      .lineToLinearHeadingRelative(new Pose2d(0, -10, Math.toRadians(-90)))
      .strafeRight(10)
      .build();

//    auto.init();
    waitForStart();
//    auto.readEnvironment();

    mecanum.followTrajectorySequence(t1);

    sleep(2000);

//    auto.stop();
  }
}
