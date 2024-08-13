package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Autonomous (group = "testing", name = "showcase")
public class AutoShowcase extends OpMode {
    Follower follower;
    private Telemetry telemetryA;
    private Path forwards;
    private PathChain curve;
    private Pose pose = new Pose(0,0,0);
    private Pose pose2 = new Pose(85, 85, 0);

    @Override
    public void init() {
    follower = new Follower(hardwareMap);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    curve = follower.pathBuilder()
            .addPath(new BezierCurve(new Point(pose), new Point(40,2, Point.CARTESIAN), new Point(55, 50, Point.CARTESIAN), new Point (40, 50, Point.CARTESIAN), new Point (85, 85, Point.CARTESIAN)))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(Math.PI), .8)
            .addPath(new BezierCurve(new Point(pose2), new Point(0, 85, Point.CARTESIAN), new Point(pose)))
            .setLinearHeadingInterpolation(Math.toRadians(Math.PI),Math.toRadians(Math.PI*2), .5)
            .build();


    //forwards = new Path(new BezierCurve(new Point(0,0, Point.CARTESIAN), new Point(0,15, Point.CARTESIAN), new Point(20,0, Point.CARTESIAN)));
    }

    @Override
    public void loop() {
    follower.update();

    telemetry.addData("x", follower.getPose().getX());
    telemetry.addData("y", follower.getPose().getY());
    telemetry.addData("heading", follower.getPose().getHeading());
    }

    @Override
    public void start(){
        follower.followPath(curve, true);
    }
}
