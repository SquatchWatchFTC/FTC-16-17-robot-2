/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop", group="1 teleop")  // @Autonomous(...) is the other common choice
//@Disabled
public class teleop extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor leftF;
    DcMotor leftB;

    DcMotor rightF;
    DcMotor rightB;

    DcMotor sweeper;
    DcMotor shooter;

    DcMotor lift;

    DcMotor red;
    DcMotor blue;

    Servo leftFlipper;
    Servo rightFlipper;
    Servo liftStopper;
    Servo ballStopper;

    boolean dPadUsed;
    boolean shooterShot;

    float leftY;
    float rightY;
    float leftT;
    float rightT;

    @Override
    public void init() {
        telemetry.addData("Version: ", "1");
        red=hardwareMap.dcMotor.get("red");
        blue=hardwareMap.dcMotor.get("blue");
        leftF = hardwareMap.dcMotor.get("leftF");
        leftB = hardwareMap.dcMotor.get("leftB");

        rightF = hardwareMap.dcMotor.get("rightF");
        rightB = hardwareMap.dcMotor.get("rightB");

        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");

        lift = hardwareMap.dcMotor.get("lift");

        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");
        liftStopper = hardwareMap.servo.get("liftStopper");
        ballStopper = hardwareMap.servo.get("ballStopper");

        leftF.setDirection(DcMotor.Direction.REVERSE);
        leftB.setDirection(DcMotor.Direction.REVERSE);
        sweeper.setDirection(DcMotor.Direction.REVERSE);

        leftFlipper.setPosition(1);
        liftStopper.setPosition(1);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y;
        rightY = gamepad1.right_stick_y;

        leftT = gamepad1.left_trigger;
        rightT = gamepad1.right_trigger;

        if (!dPadUsed) {
            if (gamepad1.right_bumper) {
                leftF.setPower(leftY * 0.5);
                leftB.setPower(leftY * 0.5);

                rightF.setPower(rightY * 0.5);
                rightB.setPower(rightY * 0.5);
            } else {
                leftF.setPower(leftY);
                leftB.setPower(leftY);

                rightF.setPower(rightY);
                rightB.setPower(rightY);
            }
        }

        if (dPadUsed) {
            if (gamepad1.dpad_up) {
                leftF.setPower(-0.25);
                leftB.setPower(-0.25);
                rightF.setPower(-0.25);
                rightB.setPower(-0.25);
            } else if (gamepad1.dpad_down) {
                leftF.setPower(0.25);
                leftB.setPower(0.25);
                rightF.setPower(0.25);
                rightB.setPower(0.25);
            } else if (gamepad1.dpad_left) {
                leftF.setPower(1);
                leftB.setPower(-1);
                rightF.setPower(1);
                rightB.setPower(-1);
            } else if (gamepad1.dpad_right) {
                leftF.setPower(-1);
                leftB.setPower(1);
                rightF.setPower(-1);
                rightB.setPower(1);
            } else {
                leftF.setPower(0);
                leftB.setPower(0);
                rightF.setPower(0);
                rightB.setPower(0);
            }
        }

        if (rightT > 0.1) {
            sweeper.setPower(-1 * rightT);
            telemetry.addData("Trigger % ", rightT);
        } else if (leftT > 0.1) {
            sweeper.setPower(leftT);
            telemetry.addData("Trigger % ", leftT);
        } else {
            sweeper.setPower(0);
        }

            if (gamepad2.a) {
                shooter.setPower(0.85);
                shooterShot = true;
            } else if (gamepad2.b) {
                shooter.setPower(0.8);
                shooterShot = true;
            } else {
                shooter.setPower(0);
            }

        if (gamepad2.x) {
            ballStopper.setPosition(0.3);
        } else {
            ballStopper.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            blue.setPower(1);
            red.setPower(0);
        } else if (gamepad2.dpad_down) {
            blue.setPower(0);
            red.setPower(1);
        }

        if (gamepad2.left_trigger > 0.9 && gamepad2.right_trigger > 0.9) {
            liftStopper.setPosition(0);
            leftFlipper.setPosition(0);
            rightFlipper.setPosition(1);
        } else {
            leftFlipper.setPosition(1);
            rightFlipper.setPosition(0);
            liftStopper.setPosition(1);
        }

            if (gamepad2.right_bumper) {
                lift.setPower(1);
            } else if (gamepad2.left_bumper) {
                lift.setPower(-0.5);
            } else {
                lift.setPower(0);
            }

        if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
            dPadUsed = true;
        } else {
            dPadUsed = false;
        }
    }

    @Override
    public void stop() {}

}
