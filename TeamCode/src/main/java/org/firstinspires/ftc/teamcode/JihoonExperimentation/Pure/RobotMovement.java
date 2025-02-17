package org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure;

import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.JihoonExperimentation.Pure.MathFunctions.lineCircleIntersection;

import com.qualcomm.robotcore.util.Range;

import org.opencv.core.Point;

import java.util.ArrayList;

public class RobotMovement {

    //todo: make worldx, worldy be actual robot cords soon
    public static double worldXPosition = 0, worldYPosition = 0, worldAngle_rad = 0;

    //TODO: make these movement variables actually just be things that move the robot
    public static double movement_x = 0, movement_y = 0, movement_turn = 0;


    public static void followCurve(ArrayList<CurvePoint> allPoints, double followAngle){
        CurvePoint followMe = getFollowPointPath(allPoints, new Point(worldXPosition, worldYPosition), allPoints.get(0).followDistance);

        goToPosition(followMe.x, followMe.y, followMe.moveSpeed, followAngle, followMe.turnSpeed);
    }

    public static CurvePoint getFollowPointPath(ArrayList<CurvePoint> pathPoints, Point robotLocation, double followRadius){
        CurvePoint followMe = new CurvePoint(pathPoints.get(0));

        for(int i = 0; i < pathPoints.size()-1; i++){
            CurvePoint startLine = pathPoints.get(i);
            CurvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = lineCircleIntersection(robotLocation, followRadius,startLine.toPoint(), endLine.toPoint());

            double closestAngle = 100000000;

            for(Point thisIntersecion : intersections){
                double angle = Math.atan2(thisIntersecion.y - worldYPosition,thisIntersecion.x - worldXPosition);
                double deltaAngle = Math.abs(MathFunctions.AngleWrap(angle-worldAngle_rad));

                if(deltaAngle < closestAngle){
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersecion);
                }
            }
        }
        return followMe;
    }

    public static void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed){
        double distanceToTarget = Math.hypot(x -worldXPosition, y - worldYPosition);

        double absoluteAngleToTarget = Math.atan2(y-worldYPosition,x-worldXPosition);

        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget - (worldAngle_rad - Math.toRadians(90)));



        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;

        movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30),-1,1) * turnSpeed;

        if(distanceToTarget < 10){
            movement_turn = 0;
        }
    }

    public void update(){
        //update Localizer
        //set world Cords with localizer

    }
}
