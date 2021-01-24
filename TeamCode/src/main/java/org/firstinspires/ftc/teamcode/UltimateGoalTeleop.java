package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "UltimateGoalTeleop")
public class UltimateGoalTeleop extends OpMode {
    Robot robot = new Robot();

    ElapsedTime timer = new ElapsedTime();
    int wobbleGoalPosition = 0;

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {
        //mecanum drive
        double multiplier = 0.8;
        if (gamepad1.a) {
            multiplier = 0.4;
        }
        if (gamepad1.b) {
            multiplier = 0.8;
        }
        double speed = multiplier * -gamepad1.left_stick_y;
        double strafe = multiplier * gamepad1.left_stick_x;
        double turn = multiplier * -gamepad1.right_stick_x;

        robot.leftFrontMotor.setPower(speed + strafe + turn);
        robot.rightFrontMotor.setPower(speed - strafe - turn);
        robot.rightBackMotor.setPower(speed + strafe - turn);
        robot.leftBackMotor.setPower(speed - strafe + turn);

        //wobble goal
        if (gamepad1.dpad_up)
            robot.wobbleGoalMotor.setTargetPosition(30);
        if (gamepad1.dpad_down)
            robot.wobbleGoalMotor.setTargetPosition(500);
        if (gamepad1.dpad_left)
            robot.wobbleGoalMotor.setTargetPosition(470);
        if (gamepad1.dpad_right)
            robot.wobbleGoalMotor.setTargetPosition(270);
        if (gamepad1.left_bumper)
            robot.wobbleGoalServo.setPosition(0.85);
        if (gamepad1.right_bumper)
            robot.wobbleGoalServo.setPosition(0.65);

        //collector
        if (gamepad2.x)
            robot.collectorMotor.setPower(1.0);
        if (gamepad2.y)
            robot.collectorMotor.setPower(0.0);

        //delivery
        if (gamepad2.dpad_up)
            robot.deliveryLiftServo.setPosition(0.1);
        if (gamepad2.dpad_down)
            robot.deliveryLiftServo.setPosition(0);

        if (gamepad2.dpad_right) {
            robot.deliveryRingServo.setPosition(0.25);
            timer.reset();
        }
        if (timer.milliseconds() < 300 && timer.milliseconds() > 200) {
            robot.deliveryRingServo.setPosition(0.0);
        }



        telemetry.addData("wobbleGoalMotor Position: ", robot.wobbleGoalMotor.getTargetPosition());
        telemetry.addData("wobbleGoalServo Position: ", robot.wobbleGoalServo.getPosition());
//        telemetry.addData("leftFront Power: ", speed + strafe + turn);
//        telemetry.addData("rightFront Power: ", speed - strafe - turn);
//        telemetry.addData("rightBack Power: ", speed + strafe - turn);
//        telemetry.addData("leftBack Power: ", speed - strafe + turn);

    }

}
