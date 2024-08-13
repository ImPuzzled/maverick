package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.SingleRunAction;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
@Config
@Autonomous
public class BlueLeftSidePath extends OpMode {
    private Timer pathTimer, opmodeTimer, scanTimer;
    // IMPORTANT: y increasing is towards the backstage from the audience,
    // while x increasing is towards the red side from the blue side
    // this means that 0 heading is pointing from the blue side to the red side

    // all spike mark locations since I'm lazy
    private Pose redLeftSideLeftSpikeMark = new Pose(36+72,-47.5+72);
    private Pose redLeftSideMiddleSpikeMark = new Pose(24.5+72,-36+72);
    private Pose redLeftSideRightSpikeMark = new Pose(36+72,-24.5+72);
    private Pose redRightSideLeftSpikeMark = new Pose(36+72, 0.5+72);
    private Pose redRightSideMiddleSpikeMark = new Pose(24.5+72, 12+72);
    private Pose redRightSideRightSpikeMark = new Pose(36+72, 23.5+72);
    private Pose blueLeftSideLeftSpikeMark = new Pose(41, 95);
    private Pose blueLeftSideMiddleSpikeMark = new Pose(-24.5+72, 12+72);
    private Pose blueLeftSideRightSpikeMark = new Pose(-36+72, 0.5+72);
    private Pose blueRightSideLeftSpikeMark = new Pose(-36+72, -24.5+72);
    private Pose blueRightSideMiddleSpikeMark = new Pose(-24.5+72, -36+72);
    private Pose blueRightSideRightSpikeMark = new Pose(-36+72, -47.5+72);

    // backdrop april tag locations
    private Pose blueLeftBackdrop = new Pose(-42.875+72, 127);
    private Pose blueMiddleBackdrop = new Pose(-36.75+72, 60.75+72);
    private Pose blueRightBackdrop = new Pose(-30.75+72, 60.75+72);
    private Pose redLeftBackdrop = new Pose(30.75+72, 60.75+72);
    private Pose redMiddleBackdrop = new Pose(36.75+72, 60.75+72);
    private Pose redRightBackdrop = new Pose(42.875+72, 60.75+72);

    // white pixel stack locations
    private Pose redOuterStack = new Pose(36+72, -72+72);
    private Pose redMiddleStack = new Pose(24+72, -72+72);
    private Pose redInnerStack = new Pose(12+72,-72+72);
    private Pose blueInnerStack = new Pose(-12+72,-72+72, Math.toRadians(270));
    private Pose blueMiddleStack = new Pose(-24+72, -72+72);
    private Pose blueOuterStack = new Pose(-36+72, -72+72);
    //Set Poses to be addressed later
    private Pose spikeMarkGoalPose, initialBackdropGoalPose, firstCycleStackPose, firstCycleBackdropGoalPose, secondCycleStackPose, secondCycleBackdropGoalPose;
    //Start Position
    private Pose startPose = new Pose(144-(63+72), 12+72, Math.toRadians(180));

    private Follower follower;
//Define Paths
    private Path scoreSpikeMark, initialScoreOnBackdrop, firstCycleToStack, firstCycleScoreOnBackdrop, secondCycleToStack, secondCycleScoreOnBackdrop;
    private PathChain firstCycleStackGrab, secondCycleStackGrab;

    private int pathState;
    public void setBackdropGoalPose() {
        //Left Path
        spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX() - 3, blueLeftSideLeftSpikeMark.getY(), Math.toRadians(270));
        initialBackdropGoalPose = new Pose(blueLeftBackdrop.getX() + 3, blueLeftBackdrop.getY()-14, Math.toRadians(270));
    }
    public void buildPaths(){
        Point scoreSpikeMarkMidPoint;

        //Left Placement
        scoreSpikeMarkMidPoint = new Point(28, 82+1, Point.CARTESIAN);
// Actual Scoring path
        scoreSpikeMark = new Path(new BezierCurve(new Point(startPose), scoreSpikeMarkMidPoint, new Point(spikeMarkGoalPose)));
        scoreSpikeMark.setConstantHeadingInterpolation(startPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(3);
//Left Score on Backdrop
        initialScoreOnBackdrop = new Path(new BezierLine(scoreSpikeMark.getLastControlPoint(), new Point(initialBackdropGoalPose)));
        initialScoreOnBackdrop.setConstantHeadingInterpolation(initialBackdropGoalPose.getHeading());
        scoreSpikeMark.setPathEndTimeoutConstraint(1);
//first cycle to stack
        firstCycleToStack = new Path(new BezierCurve(new Point(initialBackdropGoalPose), new Point(70, 107, Point.CARTESIAN), new Point(70, 33.5, Point.CARTESIAN), new Point(60-1.7, 12-3.5, Point.CARTESIAN)));
        firstCycleToStack.setConstantHeadingInterpolation(Math.toRadians(270));
        firstCycleToStack.setPathEndTimeoutConstraint(1);
//first cycle to score
        firstCycleScoreOnBackdrop = new Path(new BezierCurve(new Point(60-1.7, 12-3.5, Point.CARTESIAN), new Point(70, 33.5, Point.CARTESIAN), new Point(70, 107, Point.CARTESIAN), new Point(47, 114 +2, Point.CARTESIAN)));
        firstCycleScoreOnBackdrop.setConstantHeadingInterpolation(Math.toRadians(270));
        firstCycleScoreOnBackdrop.setPathEndTimeoutConstraint(1);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(scoreSpikeMark);
                setPathState(11);
                break;
            case 11:
                scoreSpikeMark.setLinearHeadingInterpolation(startPose.getHeading(), initialBackdropGoalPose.getHeading());
                setPathState(12);
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 2500) {
                    follower.followPath(initialScoreOnBackdrop);
                    setPathState(13);
                }
                break;
            case 13:
                boolean t = true;
                if (t == true){
                    setPathState(14);
                }
                break;
            case 14: //the first cycle to the stack
                if (follower.getCurrentTValue() > .99) {
                    follower.followPath(firstCycleToStack);
                    setPathState(15);
                }
                break;
            case 15:
                firstCycleToStack.setLinearHeadingInterpolation(initialBackdropGoalPose.getHeading(), blueInnerStack.getHeading());
                pathTimer.resetTimer();
                setPathState(16);
                break;
            case 16:
                if (follower.getCurrentTValue() > .99){
                    follower.followPath(firstCycleScoreOnBackdrop, true);
                    setPathState(17);
                }
                break;
            case 17:

                firstCycleScoreOnBackdrop.setLinearHeadingInterpolation(initialBackdropGoalPose.getHeading(), blueInnerStack.getHeading());
                setPathState(18);
                break;
            case 18:

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

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        //telemetry.update();
    }

    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        scanTimer = new Timer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

    }

    @Override
    public void start() {
        setBackdropGoalPose();
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
    }

    @Override
    public void stop() {
    }
}
