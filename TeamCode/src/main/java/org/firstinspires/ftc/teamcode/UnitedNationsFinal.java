package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="United Nation Final", group = "Linear Op Mode")

public class UnitedNationsFinal extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        DcMotor motorBasket = hardwareMap.get(DcMotor.class, "motorBasket");
        DcMotor motorSlide = hardwareMap.get(DcMotor.class, "motorSlide");
        Servo servoGripper = hardwareMap.get(Servo.class, "servo0");
        Servo servoRotator = hardwareMap.get(Servo.class, "servo1");
        DcMotor gripperMover = hardwareMap.get(DcMotor.class, "gripperMover");

        //init variables
        double armPower;
        double armReversePower;
        double slidePower;
        double slideReversePower;



        //Need to reverse one of the other wheels since new config
        //Need to reverse one of the the motors that lift the arm
        //Potentially need to reverse the arm.

        //Wheels
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);





        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;



            //Extending arm

            armPower = gamepad2.right_trigger ;
            armReversePower = gamepad2.left_trigger ;
            motorBasket.setPower(-armPower);
            motorBasket.setPower(armReversePower);

            //Lifting up the arm

            slidePower = gamepad1.right_trigger;
            slideReversePower = gamepad1.left_trigger;
            motorSlide.setPower(-slidePower);
            motorSlide.setPower(slideReversePower);




            //Gripper control --> Need to edit values
            if(gamepad2.a) {
                servoRotator.setPosition(0.7);
            } else if (gamepad2.b) {
                servoRotator.setPosition(0.3);
            }

            if(gamepad1.x) {
                servoGripper.setPosition(1);
            } else if (gamepad1.y) {
                servoGripper.setPosition(0.5);
            }


            // uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y * 0.5;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * 0.5;
            double yaw     =  gamepad1.right_stick_x * 0.5;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            //core hex motor code
            double spin = -gamepad2.right_stick_y * 0.25;

            gripperMover.setPower(spin);



            //Position of the Encoders

            double position1 = motorSlide.getCurrentPosition();

            //Target position of encoders



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm Power", armPower);
            telemetry.addData("Reverse arm Power", armReversePower);
            telemetry.addData("Encoder Position", position1);
            telemetry.addData("Slide Position", motorSlide.getPower());
            telemetry.addData("Servo Position", servoGripper.getPosition());
            telemetry.addData("Servo Power", servoRotator.getPosition());


            telemetry.update();

        }

    }}

//re-add pushh




