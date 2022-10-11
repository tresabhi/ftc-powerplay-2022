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

  public enum Side {
    FIRST,
    SECOND,
    THIRD,
    NONE
  }
  Side side;

  static final float COLOR_RANGE = 10;

  static final Scalar HSV_HIGH_1 = new Scalar(0 + COLOR_RANGE / 2, 255, 255);
  static final Scalar HSV_LOW_1 = new Scalar(0 - COLOR_RANGE / 2, 50, 50);
  static final Scalar HSV_HIGH_2 = new Scalar(60 + COLOR_RANGE / 2, 255, 255);
  static final Scalar HSV_LOW_2 = new Scalar(60 - COLOR_RANGE / 2, 80, 80);
  static final Scalar HSV_HIGH_3 = new Scalar(120 + COLOR_RANGE / 2, 255, 255);
  static final Scalar HSV_LOW_3 = new Scalar(120 - COLOR_RANGE / 2, 80, 80);

  static final Scalar COLOR_1 = new Scalar(0, 255, 255); // HSV
  static final Scalar COLOR_2 = new Scalar(60, 255, 255); // HSV
  static final Scalar COLOR_3 = new Scalar(120, 255, 255); // HSV
  static final Scalar COLOR_NONE = new Scalar(255, 255, 255); // RGB

  static final Rect ROI = new Rect(
    new Point(60, 35),
    new Point(120, 75)
  );
  double ROI_AREA = ROI.area();
  static final double COVERAGE_THRESHOLD = 0.75;

  public SleeveDetector(Telemetry telemetry) {
    this.telemetry = telemetry;
  }

  @Override
  public Mat processFrame(Mat input) {
    Mat mat = new Mat();
    Mat mat1 = new Mat();
    Mat mat2 = new Mat();
    Mat mat3 = new Mat();

    // convert from RGB to HSV because HSV is easier to read
    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
    // convert all target colors to white and the rest to black
    Core.inRange(mat, HSV_LOW_1, HSV_HIGH_1, mat1);
    Core.inRange(mat, HSV_LOW_2, HSV_HIGH_2, mat2);
    Core.inRange(mat, HSV_LOW_3, HSV_HIGH_3, mat3);

    // "cut out" the region of interest (ROI)
    Mat region1 = mat1.submat(ROI);
    Mat region2 = mat2.submat(ROI);
    Mat region3 = mat3.submat(ROI);

    // calculate coverage of corresponding colors
    double coverage1 = Core.sumElems(region1).val[0] / ROI_AREA / 255;
    double coverage2 = Core.sumElems(region2).val[0] / ROI_AREA / 255;
    double coverage3 = Core.sumElems(region3).val[0] / ROI_AREA / 255;

    region1.release();
    region2.release();
    region3.release();
    mat1.release();
    mat3.release();
    mat2.release();

    telemetry.addData("coverage1", Math.round(coverage1 * 100) + '%');
    telemetry.addData("coverage2", Math.round(coverage2 * 100) + '%');
    telemetry.addData("coverage3", Math.round(coverage3 * 100) + '%');
    telemetry.update();

    if (coverage1 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;

      Imgproc.cvtColor(mat1, mat, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_1);
    } else if (coverage2 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;

      Imgproc.cvtColor(mat2, mat, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_2);
    } else if (coverage3 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;

      Imgproc.cvtColor(mat3, mat, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_3);
    } else {
      side = Side.NONE;

      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_NONE);
    }

    return mat;
  }

  public Side getSide() {
    return side;
  }
}
