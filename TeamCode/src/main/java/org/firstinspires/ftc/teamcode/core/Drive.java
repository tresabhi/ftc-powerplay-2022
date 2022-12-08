package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive {
  public enum ExtenderLevel {
    HIGH,
    MEDIUM,
    LOW,
    GROUND
  }

  public enum ClawState {
    OPEN,
    CLOSE
  }

  HardwareMap hardwareMap;
  Telemetry telemetry;

  DcMotorEx extender;
  Servo claw;

  public static double EXTENDER_POWER = 0.5;
  public static int EXTENDER_MIN = 0;
  public static int EXTENDER_MAX = 3200;
  public static int EXTENDER_GROUND = EXTENDER_MIN;
  public static int EXTENDER_LOW = 1550;
  public static int EXTENDER_MEDIUM = 2850;
  public static int EXTENDER_HIGH = EXTENDER_MAX;
  public double extenderOffset;
  public double extenderState = EXTENDER_MIN;

  public static double CLAW_CLOSED = 1;
  public static double CLAW_OPENED = 0.3;

  public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;

    extender = hardwareMap.get(DcMotorEx.class, "extender");
    claw = hardwareMap.get(Servo.class, "claw");

    extenderOffset = extender.getCurrentPosition();
    extender.setTargetPositionTolerance(200);
    extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    updateExtender();
    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extender.setPower(EXTENDER_POWER);
  }

  public void updateExtender() {
    extenderState = Math.min(EXTENDER_MAX, Math.max(EXTENDER_MIN, extenderState));
    extender.setTargetPosition((int) (-extenderState + extenderOffset));
  }

  public void setExtenderPosition(double position) {
    extenderState = position;
    updateExtender();
  }

  public void addExtenderPosition(double position) {
    extenderState += position;
    updateExtender();
  }

  public void setExtenderLevel(ExtenderLevel level) {
    if (level == ExtenderLevel.HIGH) {
      setExtenderPosition(EXTENDER_HIGH);
    } else if (level == ExtenderLevel.MEDIUM) {
      setExtenderPosition(EXTENDER_MEDIUM);
    } else if (level == ExtenderLevel.LOW) {
      setExtenderPosition(EXTENDER_LOW);
    } else if (level == ExtenderLevel.GROUND) {
      setExtenderPosition(EXTENDER_GROUND);
    }
  }

  public void setClawState(ClawState state) {
    claw.setPosition(state == ClawState.OPEN ? CLAW_OPENED : CLAW_CLOSED);
  }
}
