/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode.OpenCVStuff;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class TheAutonomous extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;



    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;

    private Servo Intake;

    private DcMotor armMotor = null;

    private int low = DriveConstants.low;
    private int med = DriveConstants.med;
    private int high = DriveConstants.high;
    private int pickup = DriveConstants.pickup;
    private int ground = DriveConstants.ground;
    private int stack1 = 650;

    private double closePos = 0.4;
    private double openPos = 0.28;
    static final double upSpeed = 0.9;
    static final double downSpeed = -0.3;




    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

//Tag IDs


    int Left = 1;
    int Middle = 2;
    int Right = 3;

    boolean hasNotSeenTag = true;


    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode ()
    {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // Retrieve the needed information about each motor from the configuration.
        FL  = hardwareMap.get(DcMotor.class, "frontLeft");
        FR  = hardwareMap.get(DcMotor.class, "frontRight");
        BL = hardwareMap.get(DcMotor.class, "backLeft");
        BR = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "armVert");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake = hardwareMap.servo.get("Intake");


        //FL.setDirection(DcMotor.Direction.REVERSE);
        //BL.setDirection(DcMotor.Direction.REVERSE);
        //rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //armMotor.setDirection(DcMotor.Direction.FORWARD);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        waitForStart();
        if (isStopRequested()) return;

        Trajectory lowPole1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(41.2)
                .build();
        Trajectory lowPole2 = drive.trajectoryBuilder(lowPole1.end())
                .forward(5)
                .build();
        Trajectory lowPole3 = drive.trajectoryBuilder(lowPole2.end())
                .back(5)
                .build();
        Trajectory strafeToCup = drive.trajectoryBuilder(lowPole3.end())
                .strafeLeft(9.5)
                .build();
        Trajectory forwardToCup = drive.trajectoryBuilder(strafeToCup.end())
                .forward(27.5)
                .build();
        Trajectory backFromCup = drive.trajectoryBuilder(forwardToCup.end())
                .back(27.5)
                .build();
        //turn
        Trajectory toMidGoal = drive.trajectoryBuilder(new Pose2d(39,-12, Math.toRadians(-135)))
                .forward(12)
                .build();
        Trajectory backAfter = drive.trajectoryBuilder(toMidGoal.end())
                .back(12)
                .build();
        //turn to park
        Trajectory forwardToPark = drive.trajectoryBuilder(new Pose2d(39,-12, Math.toRadians(-90)))
                .forward(24)
                .build();

        //Case 1
        Trajectory strafeP1 = drive.trajectoryBuilder(forwardToPark.end())
                .strafeRight(28)
                .build();
        //Case 2

        //Case 3
        Trajectory strafeP3 = drive.trajectoryBuilder(forwardToPark.end())
                .strafeLeft(24)
                .build();


        /* Update the telemetry */


       /* if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }*/



        // BEGIN PROGRAMMING TAG-DEPENDENT INSTRUCTIONS
        int PARK = 0;


        switch (tagOfInterest.id) {
            case 1:
                PARK = 1;
                break;
            case 2:
                PARK = 2;
                break;
            case 3:
                PARK = 3;
                break;
            default:
                PARK = 3;
                break;


        }
        telemetry.addData("", PARK);
        telemetry.update();



        sleep(500);


        if(PARK == 1){
            Intake.setPosition(closePos);
            sleep(700);
            armMotor.setTargetPosition(ground);
            armMotor.setPower(upSpeed);
            sleep(300);
            drive.followTrajectory(lowPole1);
            sleep(100);
            armMotor.setTargetPosition(low);
            sleep(300);
            drive.followTrajectory(lowPole2);
            sleep(100);
            Intake.setPosition(openPos);
            sleep(200);
            drive.followTrajectory(lowPole3);
            sleep(50);

            //go to stack and pick up
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armMotor.setTargetPosition(stack1);
            armMotor.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            Intake.setPosition(closePos);
            sleep(400);
            armMotor.setTargetPosition(med);
            armMotor.setPower(upSpeed);
            sleep(600);

            //go to mid goal and score
            drive.followTrajectory(backFromCup);
            sleep(500);
            drive.turn(Math.toRadians(-135));
            armMotor.setTargetPosition(med);
            armMotor.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            Intake.setPosition(openPos);
            sleep(500);

            //back to park
            drive.followTrajectory(backAfter);
            armMotor.setTargetPosition(ground);
            armMotor.setPower(downSpeed);
            sleep(200);
            drive.turn(Math.toRadians(45));
            sleep(200);
            drive.followTrajectory(forwardToPark);
            sleep(600);
            drive.followTrajectory(strafeP1);
        }

        if(PARK == 2){
            Intake.setPosition(closePos);
            sleep(700);
            armMotor.setTargetPosition(ground);
            armMotor.setPower(upSpeed);
            sleep(300);
            drive.followTrajectory(lowPole1);
            sleep(100);
            armMotor.setTargetPosition(low);
            sleep(300);
            drive.followTrajectory(lowPole2);
            sleep(100);
            Intake.setPosition(openPos);
            sleep(200);
            drive.followTrajectory(lowPole3);
            sleep(50);

            //go to stack and pick up
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armMotor.setTargetPosition(stack1);
            armMotor.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            Intake.setPosition(closePos);
            sleep(400);
            armMotor.setTargetPosition(med);
            armMotor.setPower(upSpeed);
            sleep(600);

            //go to mid goal and score
            drive.followTrajectory(backFromCup);
            sleep(500);
            drive.turn(Math.toRadians(-135));
            armMotor.setTargetPosition(med);
            armMotor.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            Intake.setPosition(openPos);
            sleep(500);

            //back to park
            drive.followTrajectory(backAfter);
            armMotor.setTargetPosition(ground);
            armMotor.setPower(downSpeed);
            sleep(200);
            drive.turn(Math.toRadians(45));
            sleep(200);
            drive.followTrajectory(forwardToPark);
            sleep(600);

        }
        if(PARK == 3){
            Intake.setPosition(closePos);
            sleep(700);
            armMotor.setTargetPosition(ground);
            armMotor.setPower(upSpeed);
            sleep(300);
            drive.followTrajectory(lowPole1);
            sleep(100);
            armMotor.setTargetPosition(low);
            sleep(300);
            drive.followTrajectory(lowPole2);
            sleep(100);
            Intake.setPosition(openPos);
            sleep(200);
            drive.followTrajectory(lowPole3);
            sleep(50);

            //go to stack and pick up
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armMotor.setTargetPosition(stack1);
            armMotor.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            Intake.setPosition(closePos);
            sleep(400);
            armMotor.setTargetPosition(med);
            armMotor.setPower(upSpeed);
            sleep(600);

            //go to mid goal and score
            drive.followTrajectory(backFromCup);
            sleep(500);
            drive.turn(Math.toRadians(-135));
            armMotor.setTargetPosition(med);
            armMotor.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            Intake.setPosition(openPos);
            sleep(500);

            //back to park
            drive.followTrajectory(backAfter);
            armMotor.setTargetPosition(ground);
            armMotor.setPower(downSpeed);
            sleep(200);
            drive.turn(Math.toRadians(45));
            sleep(200);
            drive.followTrajectory(forwardToPark);
            sleep(600);
            drive.followTrajectory(strafeP3);
        }








        sleep(10000);

        sleep(5000);
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}