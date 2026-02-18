package org.firstinspires.ftc.teamcode.kronbot.utils.detection;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
public class RedSquareDetection extends OpenCvPipeline {
    private final Mat hsvMat = new Mat();
    private final Mat redMatUpper = new Mat();
    private final Mat redMatLower = new Mat();
    private final Mat redMat = new Mat();
    private final Scalar lowerRed1 = new Scalar(0, 100, 100);
    private final Scalar upperRed1 = new Scalar(10, 255, 255);
    private final Scalar lowerRed2 = new Scalar(160, 100, 100);
    private final Scalar upperRed2 = new Scalar(179, 255, 255);
    private final List<MatOfPoint> contourList = new ArrayList<>();

    private double detectedAngle = 0.0;
    private  int detectedQuadCount = 0;
    private final double MIN_SQUARE_AREA = 5000.0;

    @Override
    public Mat processFrame(Mat frame) {
        double largestArea = 0.0;
        MatOfPoint largestSquare = null;
        contourList.clear();
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsvMat, lowerRed1, upperRed1, redMatUpper);
        Core.inRange(hsvMat, lowerRed2, upperRed2, redMatLower);
        Core.add(redMatUpper, redMatLower, redMat);
        Imgproc.findContours(redMat, contourList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (MatOfPoint contour : contourList) {
            double area = Imgproc.contourArea(contour);
            if (area < 5000) {
                continue;
            }
            MatOfPoint2f contourFloat = new MatOfPoint2f(contour.toArray());
            double peri = Imgproc.arcLength(contourFloat, true);
            MatOfPoint2f approxCurve2f = new MatOfPoint2f();
            Imgproc.approxPolyDP(contourFloat, approxCurve2f, 0.02 * peri, true);
            int vertices = (int) approxCurve2f.total();
            if (vertices == 4) {
                MatOfPoint approxPolygon = new MatOfPoint(approxCurve2f.toArray());
                if (Imgproc.isContourConvex(approxPolygon)) {
                    double areaOfPolygon = Math.abs(Imgproc.contourArea(approxPolygon));
                    if (areaOfPolygon > MIN_SQUARE_AREA && areaOfPolygon > largestArea) {
                        largestArea = areaOfPolygon;
                        if (largestSquare != null) {
                            largestSquare.release();
                        }
                        largestSquare = new MatOfPoint(approxPolygon.toArray());
                        largestArea = areaOfPolygon;
                    }
                    approxPolygon.release();
                }
                contourFloat.release();
                approxCurve2f.release();
            }

        }
        if (largestSquare != null) {
            double angle = computeOrientation(largestSquare);

            Imgproc.drawContours(frame,
                    Arrays.asList(largestSquare),
                    -1,
                    new Scalar(0, 255, 0),
                    3);

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
            largestSquare.release();
        } else {
            detectedQuadCount = 0;
            detectedAngle = 0.0;
        }
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
