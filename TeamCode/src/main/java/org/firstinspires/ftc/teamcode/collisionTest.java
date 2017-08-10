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

@Autonomous(name="Collision Test", group="3 Red")  // @Autonomous(...) is the other common choice
@Disabled
public class collisionTest extends LinearOpMode implements IDataArrivalSubscriber{

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

    int currentEncoder;

    double lightPower;

    private final double COLLISION_THRESHOLD_DELTA_G = 0.2;

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

        ballStopper.setPosition(0);

        telemetry.addData("Version ", 5);
        telemetry.update();
        gyro.registerCallback(this);

        waitForStart();

        runtime.reset();

        while (true) {
            telemetry.addData("collision ", getCollisionString());
            telemetry.update();
        }
    }
}