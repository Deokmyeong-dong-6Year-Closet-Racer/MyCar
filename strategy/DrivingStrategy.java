package strategy;

import DrivingInterface.DrivingInterface.CarStateValues;

public interface DrivingStrategy {
	CarControls applyDrivingStrategy(CarStateValues sensing_info);
}
