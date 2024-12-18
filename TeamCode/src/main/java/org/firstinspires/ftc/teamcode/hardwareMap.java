package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardwareMap {
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor motorBasket = null;

    public Servo servoGripper = null;
    public Servo servoRotator = null;




    // Local OpMode members
    HardwareMap hwMap = null;
    private final ElapsedTime period = new ElapsedTime();

    //constructor
    public hardwareMap(){
    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        //Define and Initilize motors
        leftFrontMotor  = hwMap.get(DcMotor.class, "leftFrontMotor");
        leftBackMotor  = hwMap.get(DcMotor.class, "leftBackMotor");
        rightFrontMotor = hwMap.get(DcMotor.class, "rightFrontMotor");
        rightBackMotor = hwMap.get(DcMotor.class, "rightBackMotor");
        servoGripper = hwMap.get(Servo.class, "servo0");
        servoRotator = hwMap.get(Servo.class, "servo1");
        motorBasket = hwMap.get(DcMotor.class, "motorBasket");



        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);


        //Motor power to zero
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        motorBasket.setPower(0);
    }

}




