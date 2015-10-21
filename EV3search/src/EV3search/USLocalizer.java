package EV3search;

import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class USLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};
	
	/*declare variables*/
	public static int ROTATION_SPEED = 30;

	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private int delay = 40;
	private Navigation navi;
	private int errorFilter, errorFilterMax, distanceMax, wallDistance;

	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData, LocalizationType locType) {
		/*assign values*/
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		navi = new Navigation(odo);
		errorFilter = 0;
		errorFilterMax = 70;
		distanceMax = 70;
		wallDistance = 30;
	}

	public void doLocalization() {
		//double[] pos = new double[3];
		double angleA, angleB;

		if (locType == LocalizationType.FALLING_EDGE) {

			/*rotate the robot until it sees no wall*/
			double currentDistance = this.getFilteredData();
			while (currentDistance <= wallDistance) {			/*if the distance is less than wallDistance then keep rotating until it sees no wall*/
				navi.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);	
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}

			/* keep rotating until the robot sees a wall, then latch the angle*/

			while (currentDistance > wallDistance) {
				navi.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}
			angleA = odo.getAng();										/*record angleA*/
			/*witch direction and wait until it sees no wall*/
			while (currentDistance <= wallDistance) {					/*if the distance is less than wallDistance then keep rotating in the opposite direction until it sees no wall*/
				navi.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}

			/*keep rotating until the robot sees a wall, then latch the angle*/
			while (currentDistance > wallDistance) {					/*if the distance is bigger than wallDistance then  rotating in the same direction until it sees another wall*/
				navi.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}
			angleB = odo.getAng();										/*record angleB*/

			/* angleA is clockwise from angleB, so assume the average of the
			   angles to the right of angleB is 45 degrees past 'north'*/

			odo.getLeftMotor().stop(true);				/*stop the robot from turning to get more accurate reading*/
			odo.getRightMotor().stop(false);

			double deltaDegree;
			deltaDegree = (360+angleB+45-(((360+angleB-angleA)%360)/2))%360;	/*calculate the new degree which it should turn to based on angle A and B*/
			navi.turnTo(deltaDegree, true);																		/*turn to that degree*/

			/* update the odometer position (example to follow:)*/
			
			Delay.msDelay(4000);								/*delay 4 seconds to stable the robot before reset position*/
			
			odo.setPosition(new double[] { 0.0, 0.0, 90.0 }, new boolean[] { true, true, true }); /*reset position*/
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall. This
			 * is very similar to the FALLING_EDGE routine, but the robot will
			 * face toward the wall for most of it.
			 */

			double currentDistance = this.getFilteredData();
			while (currentDistance > wallDistance) {		/*if the distance is bigger than wallDistance then keep rotating until it sees a wall*/
				navi.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}

			/*keep rotating until the robot sees no wall, then latch the angle*/

			while (currentDistance <= wallDistance) {		/*if the distance is less than wallDistance then keep rotating in the same direction until it sees no wall*/
				navi.setSpeeds(ROTATION_SPEED, -ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}
			angleA = odo.getAng();											/*record angleA*/

			/*switch direction and wait until it sees a wall*/
			while (currentDistance > wallDistance) {		/*if the distance is bigger than wallDistance then keep rotating in the opposite direction until it sees a wall*/
				navi.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}

			/* keep rotating until the robot sees no wall, then latch the angle*/
			while (currentDistance <= wallDistance) {		/*if the distance is less than wallDistance then  rotating in the same direction until it sees no wall*/
				navi.setSpeeds(-ROTATION_SPEED, ROTATION_SPEED);			
				Delay.msDelay(delay);										
				currentDistance = this.getFilteredData();
			}
			angleB = odo.getAng();																							/*record angleB*/

			/* angleB is clockwise from angleA, so assume the average of the
			   angles to the right of angleA is 45 degrees past 'north'*/

			odo.getLeftMotor().stop(true);
			odo.getRightMotor().stop(false);																		/*stop the robot to get more accurate reading*/

			double deltaDegree;

			deltaDegree = (360+angleA+45-(((360+angleA-angleB)%360)/2))%360;		/*calculate the new degree which it should turn to based on angle A and B*/
			navi.turnTo(deltaDegree, true);																			/*turn to that degree*/

			/*update the odometer position (example to follow:)*/
			
			Delay.msDelay(4000);  																							/*delay 4 seconds to stable the robot before reset position*/
			
			odo.setPosition(new double[] { 0.0, 0.0, 90.0 }, new boolean[] { true, true, true }); 	/*reset posotion*/
		}
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);					/*fetch distances from sensor to array usData*/
		if (usData[0] * 100 > distanceMax && errorFilter < errorFilterMax) {	/*if distance is huge and haven't exceed the filter count, return a smaller number than distanceMax and increase the filter count*/
			errorFilter++;
			return (distanceMax-1);
		} else if (usData[0] * 100 > distanceMax && errorFilter >= errorFilterMax) {	/*if distance is huge and exceeds the filter count, return the real distance*/
			return usData[0] * 100;
		} else {
			float distance = usData[0] * 100;

			if (distance > distanceMax) {						/*set the max distance limit, so it doesn't read distance more than distanceMax*/
				distance = distanceMax;
			}
			if (distance == 0) {										/*if the sensor read zero, return the previous value to get raid of '0' error*/
				return usData[1] * 100;
			}
			errorFilter = 0;								/*reset filter count*/
			return distance;
		}
	}

}
