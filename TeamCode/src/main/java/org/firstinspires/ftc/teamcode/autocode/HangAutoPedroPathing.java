package org.firstinspires.ftc.teamcode.autocode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.subsystems.horizontalSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.spinnerSubsystem;


import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.verticalSubsystem;

@Autonomous(name = "Hang Auto", group = "Auto")
public class HangAutoPedroPathing extends OpMode {


    public spinnerSubsystem spinner;
    public verticalSubsystem vertical;
    public horizontalSubsystem horizontal;

    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(8.3669, 63.19266, Math.toRadians(180));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(35.5, 66.49541284403671, Math.toRadians(180));

    private final Pose scorePose2 = new Pose(36.75, 75, Math.toRadians(178));

    private final Pose scorePose3 = new Pose(39, 70, Math.toRadians(179));

    private final Pose scorePose4 = new Pose(38, 70, Math.toRadians(180));

    private final Pose newScorePose = new Pose(36.75, 65.1743119266055, Math.toRadians(180));

    private final Pose frontScorePose = new Pose(29.28440366972477, 65.1743119266055, Math.toRadians(180));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(59.88990825688073, 37.431192660550465, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(54.56880733944954, 16, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */

    /** Observation pose #1 **/
    private final Pose observation1 = new Pose(11.009174311926605, 30.825688073394492, Math.toRadians(0));

    private final Pose observation2 = new Pose(11.669724770642201, 13.651376146788994, Math.toRadians(0));

    private final Pose BackOut = new Pose(30.385321100917434, 13.431192660550453, Math.toRadians(0));

    private final Pose returnToObservation = new Pose(7.25, 24.44036697247706, Math.toRadians(0));

    private final Pose returnToObservation2 = new Pose(2, 24.44036697247706, Math.toRadians(0));


    /**control points */
    private final Pose preloadControlPose = new Pose(28.18348623853211, 60.19266055045871, Math.toRadians(0));
    private final Pose firstControlPose = new Pose(32.80733944954129, 43.57798165137615, Math.toRadians(0));

    private final Pose firstControlPose2 = new Pose(25.761467889908257, 38.972477064220186, Math.toRadians(0));

    private final Pose firstControlPoseReturn = new Pose(65.8348623853211, 15.19266055045872, Math.toRadians(0));
    private final Pose secondControlPose = new Pose(87.41284403669725, 26.862385321100923, Math.toRadians(0));

    private final Pose returnToScore = new Pose(21.357798165137613, 73.9816513761468, Math.toRadians(180));

    private final Pose coolSpline1 = new Pose(16.293577981651374, 49.10091743119266, Math.toRadians(0));

    private final Pose coolSpline2 = new Pose(66.05504587155964, 20.91743119266056, Math.toRadians(0));



    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabPickup1,grabPickup2, scorePickup1, scorePickup2,
             hang1, hangRepeatFront,  hangRepeatSpecimen, returnRepeat1,
             scorePreload1, backoutPose, backinPose, hang2, hang3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */



        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierCurve(new Point(startPose), /* Control Point */ new Point(preloadControlPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        scorePreload1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .addParametricCallback(0.001, () -> vertical.vhang())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(firstControlPose), new Point(firstControlPose2), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickup1Pose), new Point(firstControlPoseReturn), new Point(observation1)))
                .setConstantHeadingInterpolation(observation1.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(observation1), new Point(secondControlPose), new Point(pickup2Pose)))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(observation2)))
                .setConstantHeadingInterpolation(observation2.getHeading())
                .build();

        hang1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(returnToObservation), new Point(returnToScore), new Point(scorePose4)))
                .setLinearHeadingInterpolation(observation2.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> vertical.vhang())
                .build();

        hang2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(returnToObservation), new Point(returnToScore), new Point(newScorePose)))
                .setLinearHeadingInterpolation(observation2.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> vertical.vhang())
                .build();

        hang3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(returnToObservation), new Point(returnToScore), new Point(newScorePose)))
                .setLinearHeadingInterpolation(observation2.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.1, () -> vertical.vhang())
                .build();

        returnRepeat1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(newScorePose), new Point(coolSpline1),new Point(coolSpline2),  new Point(returnToObservation2)))
                .setLinearHeadingInterpolation(scorePose3.getHeading(), returnToObservation.getHeading())
                .build();

        hangRepeatFront = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(returnToObservation), new Point(returnToScore), new Point(newScorePose)))
                .setLinearHeadingInterpolation(observation2.getHeading(), scorePose2.getHeading())
                .addParametricCallback(0.1, () -> vertical.vhang())
                .build();

        hangRepeatSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(returnToObservation), new Point(returnToScore), new Point(newScorePose)))
                .setLinearHeadingInterpolation(1, scorePose.getHeading())
                .addParametricCallback(0.1, () -> vertical.vhang())
                .build();

        backoutPose = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observation2), new Point(BackOut)))
                .setConstantHeadingInterpolation(0)
                .build();

        backinPose = follower.pathBuilder()
                .addPath(new BezierLine(new Point(BackOut), new Point(returnToObservation)))
                .setConstantHeadingInterpolation(0)
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                /** score preload**/
                follower.followPath(scorePreload1, true);
                setPathState(1);
                break;
            case 1:
                /** go to brick 1**/
                if (!follower.isBusy()) {
//                if(follower.getPose().getX() >= 35.75) {
                    vertical.vDriveFromUp();
                    follower.followPath(grabPickup1, true);
                    setPathState(2);
                }
                break;
            case 2:
                /** bring brick 1 to obseration**/
//                if(!follower.isBusy()) {
                if (follower.getPose().getX() <= 59 && follower.getPose().getY() < 38) {
                    follower.followPath(scorePickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                /** go to brick 2**/
//                if(!follower.isBusy()) {
                if (follower.getPose().getX() <= 15) {
                    follower.followPath(grabPickup2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /** bring brick 2 to obseration**/
//                if(!follower.isBusy()) {
                if (follower.getPose().getX() >= 55 && follower.getPose().getY() < 20) {
                    vertical.vDriveFromUp();
                    follower.followPath(scorePickup2, true);
                    setPathState(5);
                }
                break;
            case 5:
                /** back out **/
//                if(!follower.isBusy()) {
                if (follower.getPose().getX() <= 15) {
                    vertical.vDriveFromUp();
                    follower.followPath(backoutPose, true);
                    setPathState(6);
                }
                break;

            case 6:
                /** back in **/
                if(!follower.isBusy()) {
//                if (follower.getPose().getX() >= 55 && follower.getPose().getY() < 20) {
                    vertical.vDriveFromUp();
                    follower.followPath(backinPose, true);
                    setPathState(7);
                }
            case 7:
                /** hang brick 1**/
                if(!follower.isBusy()) {
//                if (follower.getPose().getX() <= 9.5) {
                    vertical.vHangWall();
                    follower.followPath(hang1, true);
                    setPathState(8);
                }
                break;
            case 8:
                /** get second hang **/
                if(!follower.isBusy()) {
//                    if (follower.getPose().getX() >= 36) {
                    vertical.vDriveFromUp();
                    follower.followPath(returnRepeat1, true);
                    setPathState(9);
                }
                break;
            case 9:
                /** goto front hang 2 **/
                if(!follower.isBusy()) {
//                if (follower.getPose().getX() <= 9.5) {
                    vertical.vHangWall();
                    follower.followPath(hang2, true);
                    setPathState(11);
                }
                break;
            case 10:
                /** score hang 2 **/
//                if (follower.getPose().getX() >= 36.5) {
                if(!follower.isBusy()) {
                    follower.followPath(hangRepeatSpecimen, true);
                    setPathState(11);
                }
                break;
            case 11:
                /** get second hang **/
                if(!follower.isBusy()) {
//                    if (follower.getPose().getX() >= 36) {
                    vertical.vDriveFromUp();
                    follower.followPath(returnRepeat1, true);
                    setPathState(12);
                }
                break;
            case 12:
                /** goto front hang 3 **/
                if(!follower.isBusy()) {
//                if (follower.getPose().getX() <= 9.5) {
                    vertical.vHangWall();
                    follower.followPath(hang3, true);
                    setPathState(14);
                }
                break;
            case 13:
                /** score hang 3 **/
//                if (follower.getPose().getX() >= 36.5) {
                if(!follower.isBusy()) {
                    follower.followPath(hangRepeatSpecimen, true);
                    setPathState(14);
                }
                break;
            case 14:
                /** score hang 3 **/
//                if (follower.getPose().getX() >= 36.5) {
                if(!follower.isBusy()) {
                    vertical.vDriveFromUp();
                    follower.followPath(returnRepeat1, true);
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        spinner = new spinnerSubsystem(hardwareMap);
        vertical = new verticalSubsystem(hardwareMap);
        horizontal = new horizontalSubsystem(hardwareMap);

        // Set the claw to positions for init

        horizontal.horizontalInForever();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}