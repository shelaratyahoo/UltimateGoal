package org.firstinspires.ftc.teamcode.Samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

@TeleOp(name="T265", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    @Override
    public void init() {
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }
        slamra.setPose(startingPose);
        telemetry.addData("Starting Pose", "Initializing Pose...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Inside init_loop...");
        telemetry.update();
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null){
            telemetry.addLine("Camera Update returned null, exiting the programs...");
            telemetry.update();
            return;
        }

        // We divide by 0.0254 to convert meters to inches
        //Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() , up.pose.getTranslation().getY());
        Rotation2d rotation = up.pose.getRotation();

        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;

        //field.strokeLine(x1, y1, x2, y2);

        telemetry.addData("X & Y position", "x=%.2f, y=%.2f", (float)translation.getX(), (float)translation.getY());
        //telemetry.addData("X&Y dimensions", "x1=%.2f, y1=%.2f", (float)x1, (float)y1);
        //telemetry.addData("X&Y dimensions", "x2=%.2f, y2=%.2f", (float)x2, (float)y2);
        telemetry.update();
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}
