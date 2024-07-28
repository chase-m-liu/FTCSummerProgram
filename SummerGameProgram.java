package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class SummerGameProgram extends OpMode {
    
    private PIDController controller;
    //already tuned using FTC Dashboard. Don't Change.
    public static double p = 0.0053, i = 0, d = 0.0002;
    public static double f = -0.1;

    public static int target = 0;

    private final double ticks_in_degree = 751.8 / 180;

    //declare all the DcMotors
    private DcMotor arm_motor;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    //declare the servo
    private Servo claw;

    @Override
    public void init(){

        //create a new PID Controller
        controller = new PIDController(p, i, d);
        target = 0;

        //add data to FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //define the motors
        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBack = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackRight");
        arm_motor = hardwareMap.get(DcMotor.class, "arm");

        //define the servos
        claw = hardwareMap.get(Servo.class, "Claw");    

        //reverse the back right motor so the drive train will move properly
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //when we arent controlling the robot, all motors will break to prevent drifting
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        //set power to all motors based off of the left stick on gamepad1
        //                    forward/backward        left/right           turn right/turn left
        leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);

        //set the target for where the arm should move to based off of the left stick y position on the second gamepad
        target += gamepad2.left_stick_y * 2;

        //move the claw position to open when the x button is clicked
        if (gamepad2.x){
            claw.setPosition(0.4);
        }

        //move the claw position to close when the b button is clicked
        if (gamepad2.b){
            claw.setPosition(1);
        }
        
        controller.setPID(p, i, d);

        //get current arm position
        int armPos = arm_motor.getCurrentPosition();

        //pid calculations
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        //calculate the power needed
        double power = pid + ff;

        arm_motor.setPower(power);
        
        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
