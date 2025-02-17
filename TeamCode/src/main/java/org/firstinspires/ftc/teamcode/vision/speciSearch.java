package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.imgproc.Imgproc.Canny;
import static org.opencv.imgproc.Imgproc.HoughLinesP;
import static org.opencv.imgproc.Imgproc.MORPH_ELLIPSE;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.dilate;
import static org.opencv.imgproc.Imgproc.erode;
import static org.opencv.imgproc.Imgproc.getStructuringElement;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class speciSearch implements VisionProcessor {
    //OpenCV sim stuff
    public boolean seeColorBased;
    public boolean seePostThreshold;
    public boolean seeMorphology;

    public int thresholdOffset = 230;

    //telemetry
    Telemetry telemetry;

    //Class attributes
    public boolean isBlue = true;
    Scalar blueSpeciColor = new Scalar(70,120,200);
    Scalar redSpeciColor = new Scalar(175, 45, 15);

    Scalar activeColor;
    //matrices
    Mat colorFrame = new Mat();
    Mat groundSubmat = new Mat();
    //matrix attributes
    Size frameSize = new Size();

    public Point groundTopLeft = new Point();
    public Point groundBottomRight = new Point();

    double avg = 0.0;

    //kernal
    public int dilateSize = 1;
    public int erodeSize = 2;
    //things
    double bufferWidth = 64;
    private static final Scalar BLUE = new Scalar(0, 0, 255);

    public speciSearch(Telemetry telemetry){
        this.telemetry = telemetry;

        telemetry.addLine("t open");
    }
    public speciSearch(Telemetry telemetry, boolean isBlue){
        this.telemetry = telemetry;
        this.isBlue = isBlue;
        telemetry.addLine("t open");
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        activeColor = isBlue?blueSpeciColor:redSpeciColor;
        frameSize = frame.size();
        colorFrame = frame.clone();
        groundSubmat = colorFrame.submat(new Rect(groundTopLeft, groundBottomRight));
        avg = (Core.mean(groundSubmat).val[0]+Core.mean(groundSubmat).val[1]+Core.mean(groundSubmat).val[2])/3;
        for(int x = 0;x<frameSize.width;x++){
            for(int y = 0;y<frameSize.height;y++){
                double[] colorAtPoint = colorFrame.get(y,x);
//                double result = (255-Math.abs(colorAtPoint[0]-avg-200))/3+(255-Math.abs(colorAtPoint[1]-avg-150))/3+(255-colorAtPoint[0])/3;
                double result = (255-Math.abs(colorAtPoint[0]-activeColor.val[0]))/2.5+(255-Math.abs(colorAtPoint[1]-activeColor.val[1]))/2.5+(255-Math.abs(colorAtPoint[2]-activeColor.val[2]))/2.5-50;

//                double result = 255;
//                result -=Math.abs(colorAtPoint[0]-avg-200)/2;
//                result -=Math.abs(colorAtPoint[1]-avg-150)/2;
                double mean = (colorAtPoint[0]+colorAtPoint[1]+colorAtPoint[2])/3;
                result += Math.abs((colorAtPoint[0]-mean))/10+Math.abs(((colorAtPoint[0]-mean)))/10+Math.abs(((colorAtPoint[0]-mean)))/10;
                result = Math.max(0,Math.min(result,255));
                //case red = punish for low g value
                //case metal = punish for high blue value
                double[] out = new double[]{result, result, result, 255};
                colorFrame.put(y,x,out);
            }
        }
        Imgproc.cvtColor(colorFrame, colorFrame, Imgproc.COLOR_BGR2GRAY);

        if(seeColorBased)
            colorFrame.copyTo(frame);
        Mat postThreshold = new Mat();
        Imgproc.threshold(colorFrame, postThreshold, thresholdOffset+avg, 255, Imgproc.THRESH_BINARY);
        if(seePostThreshold)
            postThreshold.copyTo(frame);

        Mat dilateKernel = getStructuringElement(MORPH_RECT,
                new Size(2 * dilateSize + 1, 2 * dilateSize + 5),
                new Point(dilateSize, dilateSize));

        Mat erodeKernel = getStructuringElement(MORPH_RECT,
                new Size(2 * erodeSize + 1, 2 * erodeSize + 5),
                new Point(erodeSize, erodeSize));

        Mat morphTransform = new Mat();
        erode(postThreshold, morphTransform, erodeKernel);
        dilate(morphTransform, morphTransform, dilateKernel);
        if(seeMorphology)
            morphTransform.copyTo(frame);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(morphTransform, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        if(!seeMorphology) {
            for (Mat x : contours) {
                Rect boundRect = Imgproc.boundingRect(x);
                Imgproc.rectangle(frame, boundRect, BLUE, 2);
            }
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
    public double getOffset(){
        return 0.0;
    }
//    public ArrayList<double[]> getLines(){
//        return lineList;
//    }
}
