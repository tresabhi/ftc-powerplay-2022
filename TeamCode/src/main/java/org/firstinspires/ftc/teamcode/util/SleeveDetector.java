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

  static final float COLOR_RANGE = 15;
  static final float SATURATION_MAX = 255;
  static final float SATURATION_MIN = 80;
  static final float VALUE_MAX = 255;
  static final float VALUE_MIN = 80;

  static final Scalar HSV_HIGH_1 = new Scalar(5 + COLOR_RANGE / 2, SATURATION_MAX, VALUE_MAX);
  static final Scalar HSV_LOW_1 = new Scalar(5 - COLOR_RANGE / 2, SATURATION_MIN, VALUE_MIN);
  static final Scalar HSV_HIGH_2 = new Scalar(60 + COLOR_RANGE / 2, SATURATION_MAX, VALUE_MAX);
  static final Scalar HSV_LOW_2 = new Scalar(60 - COLOR_RANGE / 2, SATURATION_MIN, VALUE_MIN);
  static final Scalar HSV_HIGH_3 = new Scalar(110 + COLOR_RANGE / 2, SATURATION_MAX, VALUE_MAX);
  static final Scalar HSV_LOW_3 = new Scalar(110 - COLOR_RANGE / 2, SATURATION_MIN, VALUE_MIN);

  // in RGB
  static final Scalar COLOR_1 = new Scalar(255, 0, 0);
  static final Scalar COLOR_2 = new Scalar(0, 255, 0);
  static final Scalar COLOR_3 = new Scalar(0, 0, 255);
  static final Scalar COLOR_NONE = new Scalar(255, 255, 255);

  static final float ROI_WIDTH = 32;
  static final float ROI_HEIGHT = 64;
  static final float ROI_X = 160;
  static final float ROI_Y = 120;

  static final Rect ROI = new Rect(
    new Point(ROI_X - ROI_WIDTH / 2, ROI_Y - ROI_HEIGHT / 2),
    new Point(ROI_X + ROI_WIDTH / 2, ROI_Y + ROI_HEIGHT / 2)
  );
  double ROI_AREA = ROI.area();
  static final double COVERAGE_THRESHOLD = 0.60;

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
    mat.copyTo(mat1);
    mat.copyTo(mat2);
    mat.copyTo(mat3);

    // convert all target colors to white and the rest to black
    Core.inRange(mat1, HSV_LOW_1, HSV_HIGH_1, mat1);
    Core.inRange(mat2, HSV_LOW_2, HSV_HIGH_2, mat2);
    Core.inRange(mat3, HSV_LOW_3, HSV_HIGH_3, mat3);

    // "cut out" the region of interest (ROI)
    Mat region1 = mat1.submat(ROI);
    Mat region2 = mat2.submat(ROI);
    Mat region3 = mat3.submat(ROI);

    // calculate coverage of corresponding colors
    double coverage1 = Core.sumElems(region1).val[0] / ROI_AREA / 255;
    double coverage2 = Core.sumElems(region2).val[0] / ROI_AREA / 255;
    double coverage3 = Core.sumElems(region3).val[0] / ROI_AREA / 255;

    telemetry.addData("coverage1", Math.round(coverage1 * 100) + '%');
    telemetry.addData("coverage2", Math.round(coverage2 * 100) + '%');
    telemetry.addData("coverage3", Math.round(coverage3 * 100) + '%');
    telemetry.update();

    region1.release();
    region2.release();
    region3.release();

    if (coverage1 > COVERAGE_THRESHOLD) {
      side = Side.FIRST;
      Imgproc.cvtColor(mat1, mat1, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat1, ROI, COLOR_1);
      return mat1;
    } else if (coverage2 > COVERAGE_THRESHOLD) {
      side = Side.SECOND;
      Imgproc.cvtColor(mat2, mat2, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat2, ROI, COLOR_2);
      return mat2;
    } else if (coverage3 > COVERAGE_THRESHOLD) {
      side = Side.THIRD;
      Imgproc.cvtColor(mat3, mat3, Imgproc.COLOR_GRAY2RGB);
      Imgproc.rectangle(mat3, ROI, COLOR_3);
      return mat3;
    } else {
      side = Side.NONE;
      Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
      Imgproc.rectangle(mat, ROI, COLOR_NONE);
      return mat;
    }
  }

  public Side getSide() {
    return side;
  }
}
