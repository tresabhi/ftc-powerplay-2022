package org.firstinspires.ftc.teamcode.core;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Poser {

  public Pose2d pose2d(double x, double y, double heading) {
    return new Pose2d(y, -x, -Math.toRadians(heading));
  }

  public Pose2d pose2d(double x, double y) {
    return pose2d(x, y, 0);
  }

  public Pose2d pose2d() {
    return pose2d(0, 0, 0);
  }
}
