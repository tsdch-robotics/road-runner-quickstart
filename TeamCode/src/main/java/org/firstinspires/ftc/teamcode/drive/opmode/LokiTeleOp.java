package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;


@TeleOp(name="Loki TeleOp", group="Linear Opmode")

public class LokiTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private Servo Intake;

    private DcMotor armMotor = null;

    private int low = DriveConstants.low;
    private int med = DriveConstants.med;
    private int high = DriveConstants.high;
    private int pickup = DriveConstants.pickup;
    private int ground = DriveConstants.ground;


    @Override
    public void runOpMode() {


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive  = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "armVert");

        Intake = hardwareMap.servo.get("Intake");


        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.FORWARD);


        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (gamepad1.left_bumper){
                Intake.setPosition(.16);
            }

            if (gamepad1.right_bumper){
                Intake.setPosition(.4);
            }

            telemetry.addData("LinearSlideHeight", armMotor.getCurrentPosition());
            telemetry.update();






            if (gamepad1.a){
                armMotor.setTargetPosition(pickup);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armMotor.setPower(1);
            }

            if (gamepad1.x){
                armMotor.setTargetPosition(low);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armMotor.setPower(1);
            }

            if (gamepad1.y){
                armMotor.setTargetPosition(med);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armMotor.setPower(1);
            }

            if (gamepad1.b){
                armMotor.setTargetPosition(high);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armMotor.setPower(1);
            }

            if (gamepad1.left_trigger > 0){
                armMotor.setTargetPosition(ground);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                armMotor.setPower(1);
            }

            double max;

            double drive = gamepad1.right_stick_y;
            double turn  =  0.7 * -gamepad1.left_stick_x;
            double strafe = -gamepad1.right_stick_x;

            double denom = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);
            double frontLeftPower = (drive + turn + strafe) / denom;
            double frontRightPower = (drive - turn - strafe) / denom;
            double backLeftPower = (drive + turn - strafe) / denom;
            double backRightPower = (drive - turn + strafe) / denom;
            leftFrontDrive.setPower(frontLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            leftBackDrive.setPower(backLeftPower);
            rightBackDrive.setPower(backRightPower);

            if(gamepad1.dpad_down){
                leftFrontDrive.setPower(frontLeftPower * .5);
                rightFrontDrive.setPower(frontRightPower* .5);
                leftBackDrive.setPower(backLeftPower* .5);
                rightBackDrive.setPower(backRightPower* .5);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
            telemetry.update();
        }
    }}
