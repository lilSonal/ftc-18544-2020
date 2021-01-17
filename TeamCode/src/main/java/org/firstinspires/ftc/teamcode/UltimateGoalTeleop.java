package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "UltimateGoalTeleop")
public class UltimateGoalTeleop extends OpMode {
    Robot robot = new Robot();

    int wobbleGoalPosition = 0;

    @Override
    public void init() {
        robot.initRegular(hardwareMap);
    }

    @Override
    public void loop() {
        //mecanum drive
        double speed = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        robot.leftFrontMotor.setPower(speed + strafe + turn);
        robot.rightFrontMotor.setPower(speed - strafe - turn);
        robot.rightBackMotor.setPower(speed + strafe - turn);
        robot.leftBackMotor.setPower(speed - strafe + turn);

        //wobble goal
        wobbleGoalPosition += gamepad2.right_stick_y * 2;
        robot.wobbleGoalMotor.setTargetPosition(wobbleGoalPosition);
        robot.wobbleGoalMotor.setPower(0.2);
        if (gamepad2.dpad_down) {
            robot.wobbleGoalServo.setPosition(0.85);
        }
        if (gamepad2.dpad_up) {
            robot.wobbleGoalServo.setPosition(0.65);
        }
        telemetry.addData("wobbleGoalMotor Position: ", robot.wobbleGoalMotor.getTargetPosition());
        telemetry.addData("wobbleGoalServo Position: ", robot.wobbleGoalServo.getPosition());
//        telemetry.addData("leftFront Power: ", speed + strafe + turn);
//        telemetry.addData("rightFront Power: ", speed - strafe - turn);
//        telemetry.addData("rightBack Power: ", speed + strafe - turn);
//        telemetry.addData("leftBack Power: ", speed - strafe + turn);

    }

}
