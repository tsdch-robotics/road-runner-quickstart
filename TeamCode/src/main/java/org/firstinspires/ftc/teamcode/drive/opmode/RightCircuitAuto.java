package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous(group = "drive")
public class RightCircuitAuto extends LinearOpMode {
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //OpenCvCamera camera;
    //AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.5);

    /*
    private DcMotorEx frontLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx backRight = null;
*/
    private DcMotorEx armVert = null;

    private Servo leftHand = null;

    private ElapsedTime runtime = new ElapsedTime();
    static final int pickup = 0;
    final int highGoal = 4000;
    final int midGoal = 2800;
    final int lowGoal = 1900;
    final int stackPickUp = 600;

    static final double openPos = 0.89;
    static final double closePos = 0.7;

    private enum Direction {left, right;}

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(0));
        drive.setPoseEstimate(startPose);
        /*
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft  = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
*/
        armVert = hardwareMap.get(DcMotorEx.class, "armVert");
        leftHand = hardwareMap.get(Servo.class, "left_hand");

        armVert.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

/*
        //Set up camera
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

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id  == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
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
                .strafeLeft(40)
                .build();
        Trajectory lowPole2 = drive.trajectoryBuilder(lowPole1.end())
                .forward(4)
                .build();
        Trajectory lowPole3 = drive.trajectoryBuilder(lowPole2.end())
                .back(4)
                .build();
        Trajectory strafeToCup = drive.trajectoryBuilder(lowPole3.end())
                .strafeLeft(13.5)
                .build();
        Trajectory forwardToCup = drive.trajectoryBuilder(strafeToCup.end())
                .forward(25)
                .build();
        Trajectory curveToMidGoal = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(28.5, -11.5, Math.toRadians(-90)))
                .build();
        Trajectory toMidGoal = drive.trajectoryBuilder(curveToMidGoal.end())
                .forward(3)
                .build();


        leftHand.setPosition(closePos);
        sleep(600);
        armVert.setTargetPosition(lowGoal);
        armVert.setPower(0.8);
        sleep(600);
        drive.followTrajectory(lowPole1);
        sleep(200);
        drive.followTrajectory(lowPole2);
        sleep(200);
        leftHand.setPosition(openPos);
        sleep(500);
        drive.followTrajectory(lowPole3);
        sleep(200);
        drive.followTrajectory(strafeToCup);
        sleep(200);
        armVert.setTargetPosition(stackPickUp);
        armVert.setPower(0.8);
        sleep(500);
        drive.followTrajectory(forwardToCup);
        leftHand.setPosition(closePos);
        sleep(600);
        armVert.setTargetPosition(lowGoal);
        armVert.setPower(0.8);
        sleep(600);
        drive.followTrajectory(curveToMidGoal);
        sleep(500);
        armVert.setTargetPosition(midGoal);
        armVert.setPower(0.8);
        sleep(600);
        drive.followTrajectory(toMidGoal);
        sleep(200);
        leftHand.setPosition(openPos);
        sleep(500);

        sleep(2000);
        /*
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            leftHand.setPosition(closePos);
            sleep(600);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(0.8);
            sleep(600);
            drive.followTrajectory(lowPole1);
            sleep(200);
            drive.followTrajectory(lowPole2);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(500);
            drive.followTrajectory(lowPole3);
            sleep(200);
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(0.8);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            leftHand.setPosition(closePos);
            sleep(600);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(0.8);
            sleep(600);
            drive.followTrajectory(curveToMidGoal);
            sleep(500);
            armVert.setTargetPosition(midGoal);
            armVert.setPower(0.8);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(500);

            sleep(2000);
        }
        else if (tagOfInterest.id == MIDDLE) {
            //code
        }
        else {
            //code
        }



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
         */
    }
}
