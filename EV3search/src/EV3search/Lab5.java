package EV3search;

import lejos.hardware.Sound;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Lab5 {

	/* Static Resources:
	   Left motor connected to output A
	   Right motor connected to output D
	   Ultrasonic sensor port connected to input S1
	   Color sensor port connected to input S2*/
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	private static final EV3LargeRegulatedMotor sensorMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final Port usPort = LocalEV3.get().getPort("S1");		
	private static final Port colorPort = LocalEV3.get().getPort("S4");		

	
	public static void main(String[] args) {
		
		int buttonChoice;

		/*Setup ultrasonic sensor
		   1. Create a port object attached to a physical port (done above)
		   2. Create a sensor instance and attach to port
		   3. Create a sample provider instance for the above and initialize operating mode
		   4. Create a buffer for the sensor data*/
		@SuppressWarnings("resource")							    	/*Because we don't bother to close this resource*/
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		final SampleProvider usValue = usSensor.getMode("Distance");			/*colorValue provides samples from this instance*/
		final float[] usData = new float[usValue.sampleSize()];				/*colorData is the buffer in which data are returned*/
		
		/*Setup color sensor
		   1. Create a port object attached to a physical port (done above)
		   2. Create a sensor instance and attach to port
		   3. Create a sample provider instance for the above and initialize operating mode
		   4. Create a buffer for the sensor data*/
		SensorModes colorSensor = new EV3ColorSensor(colorPort);
		final SampleProvider colorValue = colorSensor.getMode("RGB");			/*colorValue provides samples from this instance*/
		final float[] colorData = new float[colorValue.sampleSize()];			/*colorData is the buffer in which data are returned*/
				
		/*setup the odometer and display*/
		final TextLCD t = LocalEV3.get().getTextLCD();
		final Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		
		/*setup a button for convenience*/
		do {
			// clear the display
			t.clear();

			// ask the user whether the motors should drive in a square or float
			t.drawString("< Left | Right >", 0, 0);
			t.drawString("       |        ", 0, 1);
			t.drawString("  Part | Part   ", 0, 2);
			t.drawString("   one | two    ", 0, 3);
			t.drawString("       |        ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT){
			t.clear();
			odo.start();
			
			(new Thread() {
				public void run() {
					Detection od = new Detection( odo, usValue, usData, sensorMotor, colorValue, colorData);
					od.objectDetection();
					
				}
			}).start();
		}

		if (buttonChoice == Button.ID_RIGHT) {
			t.clear();
			LCDInfo lcd = new LCDInfo(odo);
			
			(new Thread() {
				public void run() { /*start driving the preset path*/
					/* perform the ultrasonic localization*/
					//USLocalizer usl = new USLocalizer(odo, usValue, usData, USLocalizer.LocalizationType.FALLING_EDGE);
					//usl.doLocalization();
					
					//Delay.msDelay(5000);
					/*perform the light sensor localization*/
					//LightLocalizer lsl = new LightLocalizer(odo, colorValue, colorData);
					//lsl.doLocalization();

					Detection od = new Detection(odo, usValue, usData, sensorMotor, colorValue, colorData);
					od.findObjects();

			
				}
			}).start();
			
		}
		
	
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
		
	}

}
