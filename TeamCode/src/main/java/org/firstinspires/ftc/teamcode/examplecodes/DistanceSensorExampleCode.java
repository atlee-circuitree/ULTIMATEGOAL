package org.firstinspires.ftc.teamcode.examplecodes;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance} illustrates how to use the REV Robotics
 * Time-of-Flight Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
 */
@TeleOp(name = "Sensor: REV2mDistance", group = "Sensor")
//@Disabled
public class DistanceSensorExampleCode extends LinearOpMode {

    private DistanceSensor distance_sensor;

    @Override
    public void runOpMode() {
        // you can use this as a regular DistanceSensor.
        distance_sensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)distance_sensor;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // generic DistanceSensor methods.
            telemetry.addData("deviceName",distance_sensor.getDeviceName() );
            telemetry.addData("range", String.format("%.01f mm", distance_sensor.getDistance(DistanceUnit.MM)));
            telemetry.addData("range", String.format("%.01f cm", distance_sensor.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f m", distance_sensor.getDistance(DistanceUnit.METER)));
            telemetry.addData("range", String.format("%.01f in", distance_sensor.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();
        }
    }

}