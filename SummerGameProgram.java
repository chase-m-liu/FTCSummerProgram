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
    //already tuned
    public static double p = 0.0053, i = 0, d = 0.0002;
    public static double f = -0.1;

    public static int target = 0;

    private final double ticks_in_degree = 751.8 / 180;

    private DcMotor arm_motor;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    private Servo claw;

    @Override
    public void init(){

        controller = new PIDController(p, i, d);
        target = 0;
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotor.class, "FrontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "FrontRight");
        leftBack = hardwareMap.get(DcMotor.class, "BackLeft");
        rightBack = hardwareMap.get(DcMotor.class, "BackRight");
        arm_motor = hardwareMap.get(DcMotor.class, "arm");

        claw = hardwareMap.get(Servo.class, "Claw");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        leftBack.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        leftFront.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        rightFront.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);
        rightBack.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);

        target += gamepad2.left_stick_y * 2;

        if (gamepad2.x){
            claw.setPosition(0.4);
        }

        if (gamepad2.b){
            claw.setPosition(1);
        }

        controller.setPID(p, i, d);

        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
