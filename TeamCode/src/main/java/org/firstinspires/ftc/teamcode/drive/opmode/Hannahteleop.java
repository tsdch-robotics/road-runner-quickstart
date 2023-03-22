package org.firstinspires.ftc.teamcode.drive.opmode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//idk what any of this is or if i need or not but i copied and pasted from OutreachBot;


@TeleOp()
public class Hannahteleop extends LinearOpMode {

    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;

    double motorSpeedL;
    double motorSpeedR;

    @Override

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        waitForStart();
        while (opModeIsActive()) {
            /*
            if (gamepad1.left_stick_y > 0) {
                frontLeft.setPower(motorSpeed);
                backLeft.setPower(motorSpeed);
            } else if (gamepad1.left_stick_y < 0) {
                frontLeft.setPower(-motorSpeed);
                backLeft.setPower(-motorSpeed);
            } else (gamepad1.left_stick_y == 0) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
            }


            if (gamepad1.right_stick_y > 0) {
                frontRight.setPower(motorSpeed);
                backRight.setPower(motorSpeed);
            } else if (gamepad1.right_stick_y < 0) {
                frontRight.setPower(-motorSpeed);
                backRight.setPower(-motorSpeed);

            } else (gamepad1.right_stick_y == 0) {
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            */

            motorSpeedL = gamepad1.left_stick_y;
            motorSpeedR = gamepad1.right_stick_y;

            frontLeft.setPower(motorSpeedL);
            frontRight.setPower(motorSpeedR);
            backLeft.setPower(motorSpeedL);
            backRight.setPower(motorSpeedR);



        }

    }

}

