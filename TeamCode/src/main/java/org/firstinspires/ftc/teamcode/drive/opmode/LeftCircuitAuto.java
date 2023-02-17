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
public class LeftCircuitAuto extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

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
    final int highGoal = 1100;
    final int midGoal = 780;
    final int lowGoal = 440;
    final int stackPickUp = 120;

    static final double openPos = 0.85;
    static final double closePos = 0.65;

    static final double upSpeed = 0.85;
    static final double downSpeed = -0.2;

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


        //Set up camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //init loop

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
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
                .forward(3.4)
                .build();
        Trajectory lowPole3 = drive.trajectoryBuilder(lowPole2.end())
                .back(3.4)
                .build();
        Trajectory strafeToCup = drive.trajectoryBuilder(lowPole3.end())
                .strafeLeft(13.5)
                .build();
        Trajectory forwardToCup = drive.trajectoryBuilder(strafeToCup.end())
                .forward(24.5)
                .build();
        Trajectory curveToMidGoal = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(28.5, -12, Math.toRadians(-90)))
                .build();
        Trajectory toMidGoal = drive.trajectoryBuilder(curveToMidGoal.end())
                .forward(2)
                .build();
        Trajectory backAfter = drive.trajectoryBuilder(toMidGoal.end())
                .back(2)
                .build();
        Trajectory turnAfter = drive.trajectoryBuilder(backAfter.end())
                .lineToSplineHeading(new Pose2d(50, -9, Math.toRadians(0)))
                .build();
        Trajectory forwardToCup2 = drive.trajectoryBuilder(turnAfter.end())
                .forward(16.5)
                .build();
        Trajectory curveToHighGoal = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(24, -11, Math.toRadians(-90)))
                .build();
        Trajectory strafeToHighGoal = drive.trajectoryBuilder(curveToHighGoal.end())
                .strafeRight(14)
                .build();
        Trajectory toHighGoal = drive.trajectoryBuilder(strafeToHighGoal.end())
                .forward(4)
                .build();
        Trajectory backHighGoal = drive.trajectoryBuilder(toHighGoal.end())
                .back(4)
                .build();


        //Case 1
        Trajectory strafeToParkC1 = drive.trajectoryBuilder(backHighGoal.end())
                .strafeLeft(14)
                .build();
        Trajectory toParkC1 = drive.trajectoryBuilder(strafeToParkC1.end())
                .forward(22)
                .build();

        //Case 2
        Trajectory strafeToParkC2 = drive.trajectoryBuilder(backAfter.end())
                .strafeLeft(38)
                .build();
        Trajectory toParkC2 = drive.trajectoryBuilder(strafeToParkC2.end())
                .forward(22)
                .build();

        //Case 3
        Trajectory strafeToParkC3 = drive.trajectoryBuilder(backAfter.end())
                .strafeLeft(38)
                .build();
        Trajectory toSquareBefore = drive.trajectoryBuilder(strafeToParkC3.end())
                .forward(22)
                .build();
        Trajectory finalStrafe = drive.trajectoryBuilder(toSquareBefore.end())
                .strafeLeft(16)
                .build();



        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //Actually do something useful
        if (tagOfInterest == null || tagOfInterest.id == LEFT) {
            //score on mid
            leftHand.setPosition(closePos);
            sleep(700);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(upSpeed);
            sleep(300);
            drive.followTrajectory(lowPole1);
            sleep(100);
            armVert.setTargetPosition(stackPickUp);
            drive.followTrajectory(lowPole2);
            sleep(100);
            leftHand.setPosition(openPos);
            sleep(200);
            drive.followTrajectory(lowPole3);
            sleep(50);

            //go to stack and pick up
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            leftHand.setPosition(closePos);
            sleep(400);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(upSpeed);
            sleep(600);

            //go to mid goal and score
            drive.followTrajectory(curveToMidGoal);
            sleep(500);
            armVert.setTargetPosition(midGoal);
            armVert.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(500);

            //back to stack and pick up
            drive.followTrajectory(backAfter);
            armVert.setTargetPosition(stackPickUp - 30);
            armVert.setPower(downSpeed);
            sleep(200);
            drive.followTrajectory(turnAfter);
            armVert.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup2);
            leftHand.setPosition(closePos);
            sleep(400);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(upSpeed);
            sleep(600);

            //to high goal and score
            drive.followTrajectory(curveToHighGoal);
            drive.followTrajectory(strafeToHighGoal);
            armVert.setTargetPosition(highGoal);
            armVert.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toHighGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(300);

            //arm down and park
            drive.followTrajectory(backHighGoal);
            sleep(400);
            armVert.setTargetPosition(stackPickUp - 60);
            armVert.setPower(downSpeed);
            sleep(700);
            drive.followTrajectory(strafeToParkC1);
            drive.followTrajectory(toParkC1);

            sleep(2000);
        } else if (tagOfInterest.id == MIDDLE) {
            //score on mid
            leftHand.setPosition(closePos);
            sleep(700);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(upSpeed);
            sleep(300);
            drive.followTrajectory(lowPole1);
            sleep(100);
            armVert.setTargetPosition(stackPickUp);
            drive.followTrajectory(lowPole2);
            sleep(100);
            leftHand.setPosition(openPos);
            sleep(200);
            drive.followTrajectory(lowPole3);
            sleep(50);

            //go to stack and pick up
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            leftHand.setPosition(closePos);
            sleep(400);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(upSpeed);
            sleep(600);

            //go to mid goal and score
            drive.followTrajectory(curveToMidGoal);
            sleep(500);
            armVert.setTargetPosition(midGoal);
            armVert.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(500);

            //back to stack and pick up
            drive.followTrajectory(backAfter);
            armVert.setTargetPosition(stackPickUp - 30);
            armVert.setPower(downSpeed);
            sleep(200);
            drive.followTrajectory(turnAfter);
            armVert.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup2);
            leftHand.setPosition(closePos);
            sleep(400);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(upSpeed);
            sleep(600);

            //to high goal and score
            drive.followTrajectory(curveToHighGoal);
            drive.followTrajectory(strafeToHighGoal);
            armVert.setTargetPosition(highGoal);
            armVert.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toHighGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(300);

            drive.followTrajectory(strafeToParkC2);
            drive.followTrajectory(toParkC2);
        } else {
            //score on mid
            leftHand.setPosition(closePos);
            sleep(700);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(upSpeed);
            sleep(300);
            drive.followTrajectory(lowPole1);
            sleep(100);
            armVert.setTargetPosition(stackPickUp);
            drive.followTrajectory(lowPole2);
            sleep(100);
            leftHand.setPosition(openPos);
            sleep(200);
            drive.followTrajectory(lowPole3);
            sleep(50);

            //go to stack and pick up
            drive.followTrajectory(strafeToCup);
            sleep(200);
            armVert.setTargetPosition(stackPickUp);
            armVert.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup);
            leftHand.setPosition(closePos);
            sleep(400);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(upSpeed);
            sleep(600);

            //go to mid goal and score
            drive.followTrajectory(curveToMidGoal);
            sleep(500);
            armVert.setTargetPosition(midGoal);
            armVert.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toMidGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(500);

            //back to stack and pick up
            drive.followTrajectory(backAfter);
            armVert.setTargetPosition(stackPickUp - 30);
            armVert.setPower(downSpeed);
            sleep(200);
            drive.followTrajectory(turnAfter);
            armVert.setPower(downSpeed);
            sleep(500);
            drive.followTrajectory(forwardToCup2);
            leftHand.setPosition(closePos);
            sleep(400);
            armVert.setTargetPosition(lowGoal);
            armVert.setPower(upSpeed);
            sleep(600);

            //to high goal and score
            drive.followTrajectory(curveToHighGoal);
            drive.followTrajectory(strafeToHighGoal);
            armVert.setTargetPosition(highGoal);
            armVert.setPower(upSpeed);
            sleep(600);
            drive.followTrajectory(toHighGoal);
            sleep(200);
            leftHand.setPosition(openPos);
            sleep(300);

            //arm down and park
            drive.followTrajectory(backHighGoal);
            sleep(400);
            armVert.setTargetPosition(stackPickUp - 60);
            armVert.setPower(downSpeed);
            sleep(700);
            drive.followTrajectory(strafeToParkC3);
            drive.followTrajectory(toSquareBefore);
            drive.followTrajectory(finalStrafe);
        }


    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
