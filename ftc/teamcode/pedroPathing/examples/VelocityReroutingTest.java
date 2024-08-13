package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.google.ftcresearch.tfod.tracking.ObjectTracker;
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
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;

@Autonomous (group="testing", name ="Reouting...")
public class VelocityReroutingTest extends OpMode {
    Follower follower;
    private Timer pathTimer, opmodeTimer, scanTimer;
    private Telemetry telemetryA;
    private Path line, secondLine;
    private PoseUpdater poseUpdater;
    private Pose pose = new Pose(0,0,0);
    private Pose pose2 = new Pose(85, 85, 0);
    private Pose startPose = new Pose(0, 0, 0);
    private Pose secondPose = new Pose(0, 0, 0);
    private int pathState;

    public void buildPaths() {
        follower = new Follower(hardwareMap);
            telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetryA.update();
        line = new Path(new BezierLine(new Point(startPose), new Point(80, 0, Point.CARTESIAN)));
        secondLine = new Path(new BezierCurve(new Point(follower.getPose()), new Point(follower.getPose().getX()-24, follower.getPose().getY()+40, Point.CARTESIAN), new Point(follower.getPose().getX()+25, follower.getPose().getY()+40, Point.CARTESIAN), new Point(11 0, 0, Point.CARTESIAN)));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(line);
                setPathState(11);
                break;
            case 11:
                line.setLinearHeadingInterpolation(startPose.getHeading(), secondPose.getHeading());
                setPathState(12);
                pathTimer.resetTimer();

                break;
            case 12:
                if (follower.isBusy()){
                    if (follower.getVelocityMagnitude() < 15){
                        if (pathTimer.getElapsedTime() > 2000){
                            setPathState(14);

                        }
                    }
                } else {
                    setPathState(13);
                }
                break;
            case 13:
                setPathState(-1);
                break;
            case 14:
                follower.followPath(secondLine, true);
                secondLine.setConstantHeadingInterpolation(0);

                setPathState(-1);
                break;
        }

    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Case", pathState);
        telemetry.addData("Volcity", follower.getVelocityMagnitude());
    }

    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }


    @Override
    public void start(){
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop(){

    }
}
