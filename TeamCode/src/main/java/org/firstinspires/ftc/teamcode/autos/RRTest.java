package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.Auto;
import org.firstinspires.ftc.teamcode.core.Poser;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Autonomous(name = "[AA]RRTest", group = "Auto")
public class RRTest extends LinearOpMode {

  @Override
  public void runOpMode() throws InterruptedException {
    Auto auto = new Auto(hardwareMap, telemetry);
    SampleMecanumDrive mecanumDrive = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence t1 = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
      .forward(10)
//      .strafeRight(20)
//      .forward(38)
//      .turn(Math.toRadians(90))
      .build();

    auto.init();
    waitForStart();
    auto.readEnvironment();

    mecanumDrive.followTrajectorySequence(t1);

    sleep(2000);
    auto.stop();
  }
}
