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
    GROUND,
    ABOVE_GROUND,
    STACK_5,
    STACK_4,
    STACK_3
  }

  public enum ClawState {
    OPEN,
    CLOSE
  }

  HardwareMap hardwareMap;
  Telemetry telemetry;

  public DcMotorEx extender;
  Servo claw;

  public static double EXTENDER_POWER = 1;
  public static int EXTENDER_MIN = 0;
  public static int EXTENDER_MAX = 3800;
  public static int EXTENDER_GROUND = 0;
  public static int EXTENDER_ABOVE_GROUND = 250;
  public static int EXTENDER_LOW = 1650;
  public static int EXTENDER_MEDIUM = 2750;
  public static int EXTENDER_HIGH = EXTENDER_MAX;
  public static int EXTENDER_STACK_5 = 600;
  public static int EXTENDER_STACK_4 = 450;
  public static int EXTENDER_STACK_3 = 300;
  public double extenderState = EXTENDER_MIN;

  public static double CLAW_CLOSED = 0.95;
  public static double CLAW_OPENED = 0.6;

  public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
    this.hardwareMap = hardwareMap;
    this.telemetry = telemetry;

    extender = hardwareMap.get(DcMotorEx.class, "extender");
    claw = hardwareMap.get(Servo.class, "claw");

    extender.setTargetPositionTolerance(200);
    extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    updateExtender();
    extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    extender.setPower(EXTENDER_POWER);
  }

  public void updateExtender() {
    extenderState = Math.min(EXTENDER_MAX, Math.max(EXTENDER_MIN, extenderState));
    extender.setTargetPosition((int) (-extenderState));
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
    } else if (level == ExtenderLevel.ABOVE_GROUND) {
      setExtenderPosition(EXTENDER_ABOVE_GROUND);
    } else if (level == ExtenderLevel.STACK_5) {
      setExtenderPosition(EXTENDER_STACK_5);
    } else if (level == ExtenderLevel.STACK_4) {
      setExtenderPosition(EXTENDER_STACK_4);
    } else if (level == ExtenderLevel.STACK_3) {
      setExtenderPosition(EXTENDER_STACK_3);
    }
  }

  public void setClawState(ClawState state) {
    claw.setPosition(state == ClawState.OPEN ? CLAW_OPENED : CLAW_CLOSED);
  }
}