
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {
    Robot robot = new Robot();

    ElapsedTime timer = new ElapsedTime();
    int wobbleGoalPosition = 0;
    double shooterSpeed = 0.7;
    double increment = 0.01;

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
        robot.deliveryLiftServo.setPosition(0.67);
        robot.deliveryRingServo.setPosition(0.56);
    }

    @Override
    public void loop() {
        //mecanum drive
        double multiplier = 0.8;
        if (gamepad1.a)
            multiplier = 0.4;

        if (gamepad1.b)
            multiplier = 0.8;

        double speed = multiplier * gamepad1.left_stick_y;
        double strafe = multiplier * gamepad1.left_stick_x;
        double turn = multiplier * -gamepad1.right_stick_x;

        robot.leftFrontMotor.setPower(speed + strafe + turn);
        robot.rightFrontMotor.setPower(speed - strafe - turn);
        robot.rightBackMotor.setPower(speed + strafe - turn);
        robot.leftBackMotor.setPower(speed - strafe + turn);

        //delivery
        if (gamepad2.dpad_up)
            robot.deliveryLiftServo.setPosition(0.7676);
        if (gamepad2.dpad_down)
            robot.deliveryLiftServo.setPosition(0.67);

        if (gamepad2.dpad_right) {
            robot.deliveryRingServo.setPosition(0.79);
            timer.reset();
        }
        if (timer.milliseconds() < 300 && timer.milliseconds() > 200) {
            robot.deliveryRingServo.setPosition(0.54);
        }
        //shooter
        if (gamepad2.a)
            robot.launchingWheel.setPower(-shooterSpeed);
        if (gamepad2.b)
            robot.launchingWheel.setPower(0);
        if (gamepad2.right_bumper)
        {
            if (timer.milliseconds() > 100) {
                shooterSpeed += increment;
                timer.reset();
            }
        }
        if (gamepad2.left_bumper)
        {
            if (timer.milliseconds() > 100) {
                shooterSpeed -= increment;
                timer.reset();
            }
        }
        telemetry.addData("speed: ", shooterSpeed);
        telemetry.addData("deliveryRingServo Position: ", robot.deliveryRingServo.getPosition());
        telemetry.addData("deliveryLiftServo Position: ", robot.deliveryLiftServo.getPosition());
    }
}