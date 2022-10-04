package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SleeveDetector extends OpenCvPipeline {
  Telemetry telemetry;
  Mat mat = new Mat();

  public enum Side {
    FIRST,
    SECOND,
    THIRD,
    NONE
  }
  Side side;

  static final Scalar HSV_HIGH_1 = new Scalar(0, 255, 255);
  static final Scalar HSV_LOW_1 = new Scalar(0, 80, 80);
  static final Scalar HSV_HIGH_2 = new Scalar(60, 255, 255);
  static final Scalar HSV_LOW_2 = new Scalar(60, 80, 80);
  static final Scalar HSV_HIGH_3 = new Scalar(120, 255, 255);
  static final Scalar HSV_LOW_3 = new Scalar(120, 80, 80);

  static final Scalar COLOR_1 = new Scalar(0, 255, 255);
  static final Scalar COLOR_2 = new Scalar(60, 255, 255);
  static final Scalar COLOR_3 = new Scalar(120, 255, 255);

  static final Rect ROI = new Rect(
    new Point(60, 35),
    new Point(120, 75)
  );
  static final double COVERAGE_THRESHOLD = 0.75;

  public SleeveDetector(Telemetry telemetry) {
    this.telemetry = telemetry;
  }

  @Override
  public Mat processFrame(Mat input) {
    Mat mat1 = new Mat();
    Mat mat2 = new Mat();
    Mat mat3 = new Mat();

    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
    Core.inRange(mat, HSV_LOW_1, HSV_HIGH_1, mat1);
    Core.inRange(mat, HSV_LOW_2, HSV_HIGH_2, mat2);
    Core.inRange(mat, HSV_LOW_3, HSV_HIGH_3, mat3);

    Mat region1 = mat1.submat(ROI);
    Mat region2 = mat2.submat(ROI);
    Mat region3 = mat3.submat(ROI);

    double ROIArea = ROI.area();
    double coverage1 = Core.sumElems(region1).val[0] / ROIArea / 255;
    double coverage2 = Core.sumElems(region2).val[0] / ROIArea / 255;
    double coverage3 = Core.sumElems(region3).val[0] / ROIArea / 255;

    region1.release();
    region2.release();
    region3.release();

    telemetry.addData("coverage1", Math.round(coverage1 * 100) + '%');
    telemetry.addData("coverage2", Math.round(coverage2 * 100) + '%');
    telemetry.addData("coverage3", Math.round(coverage3 * 100) + '%');
    telemetry.update();

    if (coverage1 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;

      Imgproc.cvtColor(region1, mat, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_1);
    } else if (coverage2 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;

      Imgproc.cvtColor(region2, mat, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_2);
    } else if (coverage3 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;

      Imgproc.cvtColor(region3, mat, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_3);
    } else {
      side = Side.NONE;
    }

    return mat;
  }

  public Side getSide() {
    return side;
  }
}
