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

    public Servo servoGripper1 = null;
    public Servo servoGripper2 = null;



    // Local OpMode members
    HardwareMap hwMap = null;
    private final ElapsedTime period = new ElapsedTime();

    //constructor
    public hardwareMap(){
    }

    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        //Define and Initilize motors
        leftFrontMotor  = hwMap.get(DcMotor.class, "left front");
        leftBackMotor  = hwMap.get(DcMotor.class, "left back");
        rightFrontMotor = hwMap.get(DcMotor.class, "right front");
        rightBackMotor = hwMap.get(DcMotor.class, "right back");
        servoGripper1 = hwMap.get(Servo.class, "servo2");
        servoGripper2 = hwMap.get(Servo.class, "servo3");


        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);


        //Motor power to zero
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

}




