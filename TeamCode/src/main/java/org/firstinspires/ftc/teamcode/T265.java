package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class T265 {

    // this object should be a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;
    private ElapsedTime runtime;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public T265(HardwareMap mainHardwareMap, Telemetry mainTelemetry, ElapsedTime elapsedTime)
    {
        hardwareMap = mainHardwareMap;
        telemetry = mainTelemetry;
        runtime = elapsedTime;
    }


    public void  Initialize() {
        if(IsStarted())
        {
            Stop();
        }
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        slamra.setPose(startingPose);
        Start();
        telemetry.addData("Starting", "Initializing T265Camera.");
        telemetry.update();
    }

    public void Start() {
        if(!IsStarted()) {
            slamra.start();
        }
    }
    public void Stop() {
        if(slamra != null) {
            slamra.stop();
        }
    }
    public void Free(){
        if(slamra !=null){
            slamra.free();
        }
    }

    public boolean IsStarted() { return (slamra != null && slamra.isStarted());}

    public boolean IsGreenZone()
    {
        double xPosition = 0;
        double yPosition = 0;
        Rotation2d rotation;
        boolean result = false;

        final double backZone_X_Start = 50;
        final double backZone_X_End = 80;
        final double backZone_Y_Start = 15;
        final double backZone_Y_End = 35;
        final double backZone_Degrees_Start = 1;
        final double backZone_Degrees_End = 30;

        if(slamra == null)
        {
            return false;
        }
        if(!slamra.isStarted())
        {
            Start();
        }
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null){
            telemetry.addLine("IsGreenZone -- Camera Update returned null, exiting the programs...");
            telemetry.update();
            return result;
        }

        // We divide by 0.0254 to convert meters to inches
        xPosition = up.pose.getTranslation().getX() / 0.0254;
        yPosition = up.pose.getTranslation().getY() / 0.0254;
        rotation = up.pose.getRotation();
        double rotationDegree = rotation.getDegrees();

        if(xPosition >= backZone_X_Start && xPosition <= backZone_X_End &&
                yPosition >= backZone_Y_Start && yPosition <= backZone_Y_End &&
                rotationDegree >= backZone_Degrees_Start && rotationDegree <= backZone_Degrees_End)
        {
            result = true;
        }
        telemetry.addData(" -- Green Zone -- ", "x=%.2f, y=%.2f, degree=%.2f", (float)xPosition, (float)yPosition, (float)rotationDegree);
        telemetry.update();
        return result;
    }
    public void GetXYPosition()
    {
        final int robotRadius = 9; // inches
        double xPosition = 0;
        double yPosition = 0;
        Rotation2d rotation;

        if(slamra == null)
        {
            return;
        }
        if(!slamra.isStarted())
        {
            return;
        }
        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null){
            telemetry.addLine("Camera Update returned null, exiting the programs...");
            telemetry.update();
            return;
        }

        // We divide by 0.0254 to convert meters to inches
        xPosition = up.pose.getTranslation().getX() / 0.0254;
        yPosition = up.pose.getTranslation().getY() / 0.0254;
        rotation = up.pose.getRotation();
        double rotationDegree = rotation.getDegrees();

//        Translation2d translation = new Translation2d(xPosition, yPosition);
//        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
//        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
//        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;

        //field.strokeLine(x1, y1, x2, y2);

        //telemetry.addData("X & Y position", "x=%.2f, y=%.2f", (float)translation.getX(), (float)translation.getY());
        telemetry.addData(" -- X -- Y -- Degree -- ", "x=%.2f, y=%.2f, degree=%.2f", (float)xPosition, (float)yPosition, (float)rotationDegree);
//        telemetry.addData("X1 & Y1 dimensions", "x1=%.2f, y1=%.2f", (float)x1, (float)y1);
//        telemetry.addData("X2 & Y2 dimensions", "x2=%.2f, y2=%.2f", (float)x2, (float)y2);
        telemetry.update();
    }

}
