package EV3search;

import java.lang.reflect.Array;
import java.util.*;
import lejos.hardware.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.ev3.LocalEV3;

public class Detection {
	private Odometer odo;
	private Navigation navi;
	private SampleProvider colorSensor;
	private SampleProvider usSensor;
	private EV3LargeRegulatedMotor leftMotor, rightMotor, sensorMotor;
	private final int motorStraight = 200, FILTER_OUT = 20, sensorRotate = 50;
	private int errorFilter, errorFilterMax, distanceMax, wallDistance;

	private float[] colorData, usData;
	private double d11, d12, d13, theta11, theta12, theta13, d21, d22, d23, theta21, theta22, theta23, angleA, angleB,
			blockWidth;
	private int fallingEdgeCount = 0, risingEdgeCount = 0, delay = 40;
	final TextLCD t = LocalEV3.get().getTextLCD();
	private boolean isBlock, foundObject;
	float color;

	/* private boolean firstTimeEnter; */

	public Detection(Odometer odo, SampleProvider usSensor, float[] usData, EV3LargeRegulatedMotor sensorMotor,
			SampleProvider colorSensor, float[] colorData) {
		/*
		 * this.leftMotor = leftMotor; //this.rightMotor = rightMotor;
		 * //leftMotor.setSpeed(motorStraight); /*Initialize motor rolling
		 * forward //rightMotor.setSpeed(motorStraight); //filterControl = 0;
		 * //turnFilter = 0; //lastDegree = 45; /*pi/4???? //firstTimeEnter =
		 * true;
		 */
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.sensorMotor = sensorMotor;
		this.usSensor = usSensor;
		this.usData = usData;
		errorFilter = 0;
		errorFilterMax = 5;
		distanceMax = 100;
		wallDistance = 30;
		navi = new Navigation(odo);

		/*
		 * sensorMotor = new
		 * EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
		 */
	}

	public boolean objectDetection() {
		double distance;
		while (true) {
			distance = this.getFilteredData();
			//t.drawString("D: ", 0, 0);
			//t.drawString(("" + usData[0] * 100), 3, 0);
			if (distance < 5) {
				navi.setSpeeds(0, 0);
				sensorMotor.setSpeed(50);
				sensorMotor.rotate(-90, false);
				sensorMotor.rotate(180,
						true); /* rotate and collect distance data */
				float currentDistance = this.getFilteredData();

				while (currentDistance > 15) {
					Delay.msDelay(delay);
					currentDistance = this.getFilteredData();
				}
				d11 = currentDistance;
				d12 = d11;
				theta11 = sensorMotor.getTachoCount();

				while (currentDistance < 15) {
					Delay.msDelay(delay);
					d13 = currentDistance;
					theta13 = sensorMotor.getTachoCount();
					currentDistance = this.getFilteredData();

					if (currentDistance < d12) { /*
													 * minDistance and
													 * corresponding angle
													 */
						d12 = currentDistance;
						theta12 = sensorMotor.getTachoCount();
					}
				}

				angleA = Math.toRadians((Math.abs(theta11) + Math.abs(theta13)) % 360);
				blockWidth = Math.sqrt(d11 * d11 + d13 * d13 - 2 * d11 * d13 * Math.cos(
						angleA)); /* cosine law c^2 = a^2 + b^2 - 2abcosC */
				
				navi.turnTo(odo.getAng()-(theta11+Math.abs(theta13-theta11)/4), true);
				sensorMotor.rotateTo((int) 0, false);
				currentDistance = this.getFilteredData();
				while (currentDistance > 3.0) {
					navi.setSpeeds(30, 30);
					currentDistance = this.getFilteredData();
				}
				navi.setSpeeds(0, 0);
				Delay.msDelay(1000);

				color = getColor();
//				isBlock = (color < 7 && blockWidth < 12);
				isBlock = (color <= 3);

				displayInfo(theta11, theta12, theta13, d11, d12, d13, blockWidth, isBlock);
				t.drawString(""+color, 9, 7);
				return isBlock;
			} else {
				t.drawString("Move Object Closer", 0, 0);
			}
		}
	}

	public void findObjects() {
		foundObject = false;
		boolean trail = true;
		navi.turnTo(45, true);
		searchAndGo();
		while (!foundObject){
			double currentDistance = getFilteredData();
			if (currentDistance > 80){
				searchAndGo();
				trail = objectDetection();
				foundObject = trail;
			}
			if (!trail){
				navi.goForward(-5.0);
				searchAndGo();
			}
			
		}
		//TODO: grab the block and drive to destination
	}

	private void searchAndGo(){
		double distance, minDistance = 255, minAngle = odo.getAng();
		navi.setSpeeds(0, 0);
		sensorMotor.setSpeed(300);
		sensorMotor.rotateTo(-50);
		sensorMotor.setSpeed(50);
		sensorMotor.rotateTo(50, true);
		while(sensorMotor.getTachoCount() < 50){
			distance = getFilteredData();
			if(distance < minDistance){
				minDistance = distance;
				minAngle = odo.getAng() - sensorMotor.getTachoCount();
				System.out.println(minAngle);
			}
		}
		navi.turnTo(minAngle, true);
		sensorMotor.setSpeed(300);
		sensorMotor.rotateTo(0, true);
		navi.setSpeeds(80, 80);
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0); /* fetch distances from sensor */

		/*
		 * if distance is huge and haven't exceed the filter count, return a
		 * smaller number than distanceMax and increase the filter count
		 */
		if (usData[0] * 100 > distanceMax && errorFilter < errorFilterMax) {
			errorFilter++;
			return (distanceMax - 1);
		}

		/*
		 * if distance is huge and exceeds the filter count, return the real
		 * distance
		 */
		else if (usData[0] * 100 > distanceMax && errorFilter >= errorFilterMax) {
			return usData[0] * 100;
		}

		/*
		 * set the max distance limit, so it doesn't read distance more than
		 * distanceMax. If the sensor read zero, return the previous value to
		 * get raid of '0' error. Reset the filter count by setting
		 * errorFilter=0
		 */
		else {
			float distance = usData[0] * 100;
			if (distance > distanceMax) {
				distance = distanceMax;
			}
			if (distance == 0) {
				return usData[1] * 100;
			}
			errorFilter = 0;
			return distance;
		}
	}

	/* fetch light sensor data */
	private float getColor() {
		colorSensor.fetchSample(colorData, 0);
		float data = colorData[0] * 100;
		return data;
	}

	private void displayInfo(double theta11, double theta12, double theta13, double d11, double d12, double d13,
			double blockWidth, boolean isBlock) {
		t.drawString("T1: ", 0, 0);
		t.drawString("" + Math.round((float) theta11), 3, 0);
		t.drawString("T2: ", 0, 1);
		t.drawString("" + Math.round((float) theta12), 3, 1);
		t.drawString("T3: ", 0, 2);
		t.drawString("" + Math.round((float) theta13), 3, 2);
		t.drawString("D1: ", 0, 3);
		t.drawString("" + Math.round((float) d11), 3, 3);
		t.drawString("D2: ", 0, 4);
		t.drawString("" + Math.round((float) d12), 3, 4);
		t.drawString("D3: ", 0, 5);
		t.drawString("" + Math.round((float) d13), 3, 5);
		t.drawString("W: ", 0, 6);
		t.drawString("" + Math.round((float) blockWidth), 3, 6);
		t.drawString("IsBlock: ", 0, 7);
		t.drawString("" + isBlock, 3, 7);
	}
}