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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.kauailabs.navx.ftc.IDataArrivalSubscriber;

import java.text.DecimalFormat;

@Autonomous(name="Navx Red Autonomous", group="3 Red")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousRed extends LinearOpMode implements IDataArrivalSubscriber{

    private ElapsedTime runtime = new ElapsedTime();

    private final int NAVX_DIM_I2C_PORT = 1;

    DcMotor leftF;
    DcMotor leftB;

    DcMotor rightF;
    DcMotor rightB;

    DcMotor red;
    DcMotor blue;

    DcMotor shooter;

    OpticalDistanceSensor ods1;
    OpticalDistanceSensor ods2;

    private AHRS gyro;
    ColorSensor color;

    Servo ballStopper;
    Servo liftStopper;
    Servo leftFlipper;

    int currentEncoder;
    int gyroError;
    double currentTime;

    double lightPower;

    private final double COLLISION_THRESHOLD_DELTA_G = 0.43;

    double last_world_linear_accel_x;
    double last_world_linear_accel_y;
    private boolean collision_state;

    private final String COLLISION = "Collision";
    private final String NO_COLLISION = "--------";

    private long last_system_timestamp = 0;
    private long last_sensor_timestamp = 0;

    private long sensor_timestamp_delta = 0;
    private long system_timestamp_delta = 0;

    public void checkColor() throws InterruptedException {
        sleep(1000);
        if (color.red() + 50 > color.blue()) {
            currentTime = getRuntime();

            leftF.setPower(-.35);
            leftB.setPower(-.35);
            rightF.setPower(-.35);
            rightB.setPower(-.35);

            while (!collision_state && opModeIsActive() && currentTime + 1 > getRuntime()) { }

            leftF.setPower(.3);
            leftB.setPower(.3);
            rightF.setPower(.3);
            rightB.setPower(.3);

            currentEncoder = rightF.getCurrentPosition();

            while (currentEncoder + 200 > rightF.getCurrentPosition()) { }

            leftF.setPower(0);
            rightB.setPower(0);
            rightF.setPower(0);
            leftB.setPower(0);

        } else if (color.blue() - 50 > color.red()) {

            leftF.setPower(.8);
            leftB.setPower(-.8);
            rightF.setPower(.8);
            rightB.setPower(-.8);

            while (ods2.getLightDetected() < 0.3 && opModeIsActive()) { }

            leftF.setPower(0);
            rightB.setPower(0);
            rightF.setPower(0);
            leftB.setPower(0);

            leftF.setPower(0.3);
            leftB.setPower(0.3);
            rightF.setPower(-0.3);
            rightB.setPower(-0.3);

            while (gyro.getYaw() > -90) { }

            leftF.setPower(0);
            rightB.setPower(0);
            rightF.setPower(0);
            leftB.setPower(0);

            currentTime = getRuntime();

            leftF.setPower(-.35);
            leftB.setPower(-.35);
            rightF.setPower(-.35);
            rightB.setPower(-.35);

            sleep(1000);

            while (!collision_state && opModeIsActive() && currentTime + 1 > getRuntime()) { }

            currentEncoder = rightF.getCurrentPosition();

            leftF.setPower(.3);
            leftB.setPower(.3);
            rightF.setPower(.3);
            rightB.setPower(.3);

            while (currentEncoder + 200 > rightF.getCurrentPosition()) { }

            leftF.setPower(0);
            rightB.setPower(0);
            rightF.setPower(0);
            leftB.setPower(0);

        } else {
            telemetry.addData("Cannot determine color! Red:" + color.red() + " Blue:", color.blue());
            telemetry.update();
        }
    }
    private String getCollisionString() {
        return (this.collision_state ? COLLISION : NO_COLLISION);
    }

    private void setCollisionState( boolean newValue ) {
        this.collision_state = newValue;
    }

    @Override
    public void untimestampedDataReceived(long l, Object o) {

    }

    @Override
    public void timestampedDataReceived(long curr_system_timestamp, long curr_sensor_timestamp, Object o) {
        boolean collisionDetected = false;

        sensor_timestamp_delta = curr_sensor_timestamp - last_sensor_timestamp;
        system_timestamp_delta = curr_system_timestamp - last_system_timestamp;
        double curr_world_linear_accel_x = gyro.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = gyro.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;

        if ( ( Math.abs(currentJerkX) > COLLISION_THRESHOLD_DELTA_G ) ||
                ( Math.abs(currentJerkY) > COLLISION_THRESHOLD_DELTA_G) ) {
            collisionDetected = true;
        }

        setCollisionState( collisionDetected );
    }

    @Override
    public void yawReset() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        red=hardwareMap.dcMotor.get("red");
        blue=hardwareMap.dcMotor.get("blue");

        leftF = hardwareMap.dcMotor.get("leftF");
        leftB = hardwareMap.dcMotor.get("leftB");

        rightF = hardwareMap.dcMotor.get("rightF");
        rightB = hardwareMap.dcMotor.get("rightB");

        leftF.setDirection(DcMotor.Direction.REVERSE);
        leftB.setDirection(DcMotor.Direction.REVERSE);

        shooter = hardwareMap.dcMotor.get("shooter");

        gyro =  AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData);
        last_world_linear_accel_x = 0.0;
        last_world_linear_accel_y = 0.0;
        setCollisionState(false);

        color = hardwareMap.colorSensor.get("color");

        ods1 = hardwareMap.opticalDistanceSensor.get("ods1");
        ods2 = hardwareMap.opticalDistanceSensor.get("ods2");

        ballStopper = hardwareMap.servo.get("ballStopper");
        liftStopper = hardwareMap.servo.get("liftStopper");
        leftFlipper = hardwareMap.servo.get("leftFlipper");

        ballStopper.setPosition(0);

        liftStopper.setPosition(1);

        leftFlipper.setPosition(1);

        telemetry.addData("Version ", 1);
        telemetry.update();
        gyro.registerCallback(this);

        waitForStart();

        runtime.reset();

        gyro.zeroYaw();

        red.setPower(1);
        blue.setPower(0);

        currentEncoder = rightF.getCurrentPosition();

        leftF.setPower(-0.3);
        leftB.setPower(-0.3);
        rightF.setPower(-0.3);
        rightB.setPower(-0.3);

        while (rightF.getCurrentPosition() > currentEncoder - 800 && opModeIsActive()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(0.5);
        leftB.setPower(0.5);
        rightF.setPower(-0.5);
        rightB.setPower(-0.5);

        while (gyro.getYaw() > -90 && opModeIsActive()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(-0.3);
        leftB.setPower(-0.3);
        rightF.setPower(0.3);
        rightB.setPower(0.3);

        while (gyro.getYaw() < -88 && opModeIsActive()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        shooter.setPower(0.75);
        //.9 to .85
        sleep(1500);

        for (int i = 0; i < 2; i++) {
            ballStopper.setPosition(0.4);
            //.2 to .4
            sleep(300);
            ballStopper.setPosition(0);
            sleep(1000);
        }

        shooter.setPower(0);

        leftF.setPower(-0.5);
        leftB.setPower(-0.5);
        rightF.setPower(0.5);
        rightB.setPower(0.5);

        while (gyro.getYaw() < -40) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(0.3);
        leftB.setPower(0.3);
        rightF.setPower(-0.3);
        rightB.setPower(-0.3);

        while (gyro.getYaw() > -41) { }

        leftF.setPower(-0.4);
        leftB.setPower(-0.4);
        rightF.setPower(-0.4);
        rightB.setPower(-0.4);

        sleep(1000);
        currentTime = getRuntime();

        while (!collision_state && currentTime + 3 > getRuntime()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(0.4);
        leftB.setPower(0.4);
        rightF.setPower(-0.4);
        rightB.setPower(-0.4);

        while (gyro.getYaw() > -95) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(-0.3);
        leftB.setPower(-0.3);
        rightF.setPower(0.3);
        rightB.setPower(0.3);

        while (gyro.getYaw() < -92) { }

        red.setPower(0);
        blue.setPower(0);

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(.2);
        leftB.setPower(.2);
        rightF.setPower(.2);
        rightB.setPower(.2);

        currentEncoder = rightF.getCurrentPosition();

        while (currentEncoder + 75 > rightF.getCurrentPosition()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(-.8);
        leftB.setPower(.8);
        rightF.setPower(-.8);
        rightB.setPower(.8);

        currentTime = getRuntime();

        while (ods1.getLightDetected() < 0.11 && currentTime + 3 > getRuntime()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        if (ods1.getLightDetected() < 0.1) { sleep(100000); }

        telemetry.addData("Gyro: ", gyro.getYaw());
        telemetry.update();

        leftF.setPower(0.3);
        leftB.setPower(0.3);
        rightF.setPower(-0.3);
        rightB.setPower(-0.3);

        while (gyro.getYaw() > -90) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        checkColor();

        leftF.setPower(1);
        leftB.setPower(-1);
        rightF.setPower(1);
        rightB.setPower(-1);

        sleep(1200);

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(-0.4);
        leftB.setPower(-0.4);
        rightF.setPower(-0.4);
        rightB.setPower(-0.4);

        sleep(500);

        currentTime = getRuntime();

        while (!collision_state && currentTime + 1 > getRuntime()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(.2);
        leftB.setPower(.2);
        rightF.setPower(.2);
        rightB.setPower(.2);

        currentEncoder = rightF.getCurrentPosition();

        while (currentEncoder + 125 > rightF.getCurrentPosition()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(1);
        leftB.setPower(-1);
        rightF.setPower(1);
        rightB.setPower(-1);
        currentTime = getRuntime();

        while (ods1.getLightDetected() < 0.11 && currentTime + 4 > getRuntime()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        if (ods1.getLightDetected() < 0.1) { sleep(100000); }

        telemetry.addData("Gyro: ", gyro.getYaw());
        telemetry.update();

        leftF.setPower(0.3);
        leftB.setPower(0.3);
        rightF.setPower(-0.3);
        rightB.setPower(-0.3);

        while (gyro.getYaw() > -90) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        checkColor();
    }
}