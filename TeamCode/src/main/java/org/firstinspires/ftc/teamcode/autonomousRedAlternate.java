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

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.IDataArrivalSubscriber;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Alternate Red Autonomous", group="3 Red")  // @Autonomous(...) is the other common choice
//@Disabled
public class autonomousRedAlternate extends LinearOpMode implements IDataArrivalSubscriber{

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

    private final double COLLISION_THRESHOLD_DELTA_G = 0.4;

    double last_world_linear_accel_x;
    double last_world_linear_accel_y;
    private boolean collision_state;

    private final String COLLISION = "Collision";
    private final String NO_COLLISION = "--------";

    private long last_system_timestamp = 0;
    private long last_sensor_timestamp = 0;

    private long sensor_timestamp_delta = 0;
    private long system_timestamp_delta = 0;

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

        telemetry.addData("Version ", 2);
        telemetry.update();
        gyro.registerCallback(this);

        waitForStart();

        runtime.reset();

        gyro.zeroYaw();

        red.setPower(0);
        blue.setPower(1);
        sleep(10000);

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

        while (gyro.getYaw() > -135 && opModeIsActive()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(-0.3);
        leftB.setPower(-0.3);
        rightF.setPower(0.3);
        rightB.setPower(0.3);

        while (gyro.getYaw() < -135 && opModeIsActive()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        shooter.setPower(0.85);
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

        while (gyro.getYaw() < 0) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        leftF.setPower(0.25);
        leftB.setPower(0.25);
        rightF.setPower(-0.25);
        rightB.setPower(-0.25);

        while (gyro.getYaw() > 0) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);

        currentEncoder = rightF.getCurrentPosition();

        leftF.setPower(0.3);
        leftB.setPower(0.3);
        rightF.setPower(0.3);
        rightB.setPower(0.3);

        while (rightF.getCurrentPosition() < currentEncoder + 800 && opModeIsActive()) { }

        leftF.setPower(0);
        rightB.setPower(0);
        rightF.setPower(0);
        leftB.setPower(0);
    }
}