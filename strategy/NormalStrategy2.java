package strategy;

import DrivingInterface.DrivingInterface.CarStateValues;

public class NormalStrategy2 implements DrivingStrategy {
	private static final float BRAKE_FACTOR_FOR_HIGH_SPEED = 0.1f;
	private static final float STEERING_ADJUSTMENT = 0.32f;
	private static final float MAX_THROTTLE = 1.0f;
	private static final float MIDDLE_LANE_OFFSET = 1.8f;

	@Override
	public CarControls applyDrivingStrategy(CarStateValues sensingInfo) {
		CarControls controls = new CarControls();

		setThrottle(sensingInfo, controls);
		setSteering(sensingInfo, controls);
		checkAndHandleEmergencySituations(sensingInfo, controls);

		return controls;
	}

	private void setThrottle(CarStateValues sensingInfo, CarControls controls) {
		float throttleFactor = 0.9f / (Math.abs(getReferenceAngle(sensingInfo)) + 0.1f);
		throttleFactor = Math.min(throttleFactor, 0.11f);

		float setThrottle = 0.9f + throttleFactor;
		if (sensingInfo.speed < 60) {
			setThrottle = MAX_THROTTLE;
		} else if (sensingInfo.speed > 150) {
			setThrottle = 0.97f;
		}

		controls.setThrottle(setThrottle);
	}

	private void setSteering(CarStateValues sensingInfo, CarControls controls) {
		float steerFactor = calculateSteerFactor(sensingInfo.speed);
		float middleAdjustment = ((sensingInfo.to_middle - MIDDLE_LANE_OFFSET) / 80) * -1;
		float setSteering = (getReferenceAngle(sensingInfo) - sensingInfo.moving_angle) / (steerFactor + 0.001f);
		setSteering += middleAdjustment;

		controls.setSteering(setSteering);
	}

	private void checkAndHandleEmergencySituations(CarStateValues sensingInfo, CarControls controls) {
		for (int i = 0; i < sensingInfo.speed / 30; i++) {
			float forwardAngle = Math.abs(sensingInfo.track_forward_angles.get(i));
			if (forwardAngle > 45) {
				controls.setThrottle(controls.getThrottle() - BRAKE_FACTOR_FOR_HIGH_SPEED);
				if (forwardAngle > 80) {
					controls.setBrake(0.3f);
					adjustSteeringForEmergency(controls);
					break;
				}
			}
		}
	}

	private void adjustSteeringForEmergency(CarControls controls) {
		if (controls.getSteering() > 0) {
			controls.setSteering(controls.getSteering() + STEERING_ADJUSTMENT);
		} else {
			controls.setSteering(controls.getSteering() - STEERING_ADJUSTMENT);
		}
	}

	private float getReferenceAngle(CarStateValues sensingInfo) {
		int angleNum = (int) (sensingInfo.speed / 45);
		return angleNum > 0 ? sensingInfo.track_forward_angles.get(angleNum) : 0;
	}

	private float calculateSteerFactor(float speed) {
		if (speed > 120) {
			return speed * 0.62f;
		} else if (speed > 70) {
			return speed * 0.85f;
		} else {
			return speed * 1.6f;
		}
	}
}
