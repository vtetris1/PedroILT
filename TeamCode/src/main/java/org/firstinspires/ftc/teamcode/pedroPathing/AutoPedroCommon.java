package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;

public abstract class AutoPedroCommon extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    ArrayList<PoseItem> poseList = new ArrayList<PoseItem>() ;
    protected final Pose blue_far_init = new Pose(20, 122, Math.toRadians(140)); // Start Pose of our robot.
    protected final Pose blue_far_score = new Pose(48, 82, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    protected final Pose blue_1st_row_start = new Pose(50, 78, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose blue_1st_row_end = new Pose(25, 78, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose blue_2nd_row_start = new Pose(50, 54, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose blue_2nd_row_end = new Pose(25, 54, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose blue_3rd_row_start = new Pose(50, 30, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose blue_3rd_row_end = new Pose(25, 30, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose blue_parking = new Pose(41, 38, Math.toRadians(0)); //
    protected final Pose blue_gate = new Pose(18, 60, Math.toRadians(180));
    protected final Pose blue_near_init = new Pose(63, 9, Math.toRadians(90)); // Start Pose of our robot.
    protected final Pose blue_near_score = new Pose(60, 12, Math.toRadians(100)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    protected final Pose blue_near_turn_left = new Pose(57, 10, Math.toRadians(180));
    protected final Pose blue_near_corner = new Pose(22, 10, Math.toRadians(180));

    protected final Pose red_far_init = new Pose(112, 127, Math.toRadians(45)); // Start Pose of our robot.
    protected final Pose red_far_score = new Pose(88, 90, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    protected final Pose red_1st_row_start = new Pose(90, 83, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose red_1st_row_end = new Pose(120, 81, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose red_2nd_row_start = new Pose(90, 60, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose red_2nd_row_end = new Pose(116, 57, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose red_3rd_row_start = new Pose(88, 37, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    protected final Pose red_3rd_row_end = new Pose(127, 33, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    protected final Pose red_gate = new Pose(127, 65, Math.toRadians(10));
    protected final Pose red_parking = new Pose(100, 38, Math.toRadians(0));
    protected final Pose red_near_init = new Pose(73, 9, Math.toRadians(90)); // Start Pose of our robot.
    protected final Pose red_near_score = new Pose(76, 14, Math.toRadians(80)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    protected final Pose red_near_turn_left = new Pose(80, 10, Math.toRadians(0));
    protected final Pose red_near_corner = new Pose(100, 10, Math.toRadians(0));
    public abstract void setPath();
    public void pedroDrive(Pose pose1, Pose pose2) {
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
        follower.followPath(pathChain,true);
    }
    public void autonomousPathUpdate() {
        if ((poseList.size() > 1) && (pathState < (poseList.size() - 1))) {
            if (pathState == 0) {
                pedroDrive(poseList.get(0).pose, poseList.get(1).pose);
                pathState = 1;
                setPathState(pathState);
            } else {
                if (!follower.isBusy()) {
                    String action = poseList.get(pathState).action;
                    if (isDoneAction(action)) {
                        pedroDrive(poseList.get(pathState).pose, poseList.get(pathState+1).pose);
                        pathState += 1;
                        setPathState(pathState);
                    }
                }
            }
        }
    }

    public boolean isDoneAction(String action) {
        switch (action) {
            case "shoot":
                return doneShooting();
            case "intake":
                return doneIntake();
            case "sleep_and_shoot":
                return doneSleepAndShoot();
            case "open_gate":
                return doneOpenGate();
            default:
                return true;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public boolean doneShooting() {
        if(pathTimer.getElapsedTimeSeconds() > 3) {
            pathTimer.resetTimer();
            return true;
        }
        else {
            return false;
        }
    }

    public boolean doneIntake() {
        if(pathTimer.getElapsedTimeSeconds() > 2) {
            pathTimer.resetTimer();
            return true;
        }
        else {
            return false;
        }
    }

    public boolean doneOpenGate() {
        if(pathTimer.getElapsedTimeSeconds() > 2) {
            pathTimer.resetTimer();
            return true;
        }
        else {
            return false;
        }
    }

    public boolean doneSleepAndShoot() {
        if(pathTimer.getElapsedTimeSeconds() > 20) {
            telemetry.addData("In doneSleepAndShoot true", pathTimer.getElapsedTimeSeconds());
            //telemetry.update();
            pathTimer.resetTimer();
            return true;
        }
        else {
            telemetry.addData("In doneSleepAndShoot false", pathTimer.getElapsedTimeSeconds());
            //telemetry.update();
            return false;
        }
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();


    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        //buildPaths();
        setPath();
        follower.setStartingPose(poseList.get(0).pose);

    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {
    }
}
