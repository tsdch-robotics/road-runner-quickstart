package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class TestAuto1 extends LinearOpMode{
    static final double openPos = 0.89;
    static final double closePos = 0.7;
    static final int pickup = 0;
    final int highGoal = 4000;
    final int midGoal = 2800;
    final int lowGoal = 1900;
    final int stackPickUp = 600;
    private DcMotorEx armVert;
    private Servo leftHand;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(36, -64, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        armVert = hardwareMap.get(DcMotorEx.class, "armVert");
        leftHand = hardwareMap.get(Servo.class, "left_hand");

        armVert.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armVert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armVert.setDirection(DcMotorEx.Direction.REVERSE);


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
                .lineToSplineHeading(new Pose2d(28.5,-11.5,Math.toRadians(-90)))
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

    }
}
