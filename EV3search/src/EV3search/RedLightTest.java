package EV3search;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class RedLightTest {
	
	/* Static Resources:
	   Left motor connected to output A
	   Right motor connected to output D*/
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

	/*Constants*/
	public static final double WHEEL_RADIUS = 2.1;
	public static final double TRACK = 15.8;

	public static void main(String[] args) {
		int buttonChoice;

		/*some objects that need to be instantiated*/
		float[] sampleRGB = {0};
		final TextLCD t = LocalEV3.get().getTextLCD();
		
		EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S4);
		
				
		do {
			/*clear the display*/
			t.clear();
			t.drawString("Press Left/Right button "		, 0, 0);
			t.drawString("For the next light value "	, 0, 1);
			t.drawString("Escape to stop "				, 0, 4);
			t.drawString("Light Value: "				, 0, 6);
			lightSensor.getRGBMode().fetchSample(sampleRGB, 0);
			t.drawInt((int)(sampleRGB[0]*100), 0, 7);

			buttonChoice = Button.waitForAnyPress();
			} 
		while (buttonChoice != Button.ID_ESCAPE);
		
		t.clear();
		t.drawString("Press escape again "	, 0, 2);
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}