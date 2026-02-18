package org.firstinspires.ftc.teamcode.kronbot.utils.detection;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class BlueSquareDetecion extends OpenCvPipeline {

    private final Mat hsvMat = new Mat();
    private final Mat colorMaskMat = new Mat();
    private final Mat contourHierarchyMat = new Mat();
    private final List<MatOfPoint> contourList = new ArrayList<>();

    private static final double MIN_QUADRILATERAL_AREA = 10000.0;

    private static final Scalar HSV_LOWER_BLUE = new Scalar(90, 40, 100);
    private static final Scalar HSV_UPPER_BLUE = new Scalar(135, 255, 255);

    private volatile int detectedQuadCount = 0;
    private volatile double detectedAngle = 0.0;

    @Override
    public Mat processFrame(Mat frame) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, HSV_LOWER_BLUE, HSV_UPPER_BLUE, colorMaskMat);
        contourList.clear();
        Imgproc.findContours(colorMaskMat, contourList, contourHierarchyMat,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint largestQuadrilateral = null;
        double largestArea = 0.0;

        for (MatOfPoint contour : contourList) {
            MatOfPoint2f contourFloat = new MatOfPoint2f(contour.toArray());
            double peri = Imgproc.arcLength(contourFloat, true);
            MatOfPoint2f approxCurve2f = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourFloat, approxCurve2f, 0.02 * peri, true);

            int vertices = (int) approxCurve2f.total();
            if (vertices == 4) {
                MatOfPoint approxPolygon = new MatOfPoint(approxCurve2f.toArray());
                if (Imgproc.isContourConvex(approxPolygon)) {
                    double area = Math.abs(Imgproc.contourArea(approxPolygon));
                    if (area > MIN_QUADRILATERAL_AREA && area > largestArea) {
                        largestArea = area;
                        if (largestQuadrilateral != null) {
                            largestQuadrilateral.release();
                        }
                        largestQuadrilateral = approxPolygon;
                    } else {
                        approxPolygon.release();
                    }
                } else {
                    approxPolygon.release();
                }
            }

            contourFloat.release();
            approxCurve2f.release();
        }

        if (largestQuadrilateral != null) {
            double angle = computeOrientation(largestQuadrilateral);

            Imgproc.drawContours(frame,
                    Arrays.asList(largestQuadrilateral),
                    -1,
                    new Scalar(0, 255, 0),
                    3);

            // overlay angle text for EOCV-Sim
            Imgproc.putText(
                    frame,
                    String.format("Angle: %.1f", angle),
                    new Point(20, 40),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    new Scalar(0, 255, 0),
                    2
            );

            detectedQuadCount = 1;
            detectedAngle = angle;
            largestQuadrilateral.release();
        } else {
            detectedQuadCount = 0;
            detectedAngle = 0.0;
        }

        /// morphKernel.release();
        return frame;
    }

    private double computeOrientation(MatOfPoint quad) {
        MatOfPoint2f pts = new MatOfPoint2f(quad.toArray());
        RotatedRect rr = Imgproc.minAreaRect(pts);
        Size sz = rr.size;
        double angle = rr.angle;

        if (sz.width < sz.height) {
            angle += 90.0;
        }

        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;

        pts.release();
        return angle;
    }

    public int getDetectedQuadCount() {
        return detectedQuadCount;
    }

    public double getDetectedAngle() {
        return detectedAngle;
    }
}
