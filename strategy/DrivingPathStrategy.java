package strategy;

import java.util.ArrayList;

import DrivingInterface.DrivingInterface.CarStateValues;

public class DrivingPathStrategy implements DrivingStrategy {

	@Override
	public CarControls applyDrivingStrategy(CarStateValues sensingInfo) {
		CarControls controls = new CarControls();
		
//		sensing_info.to_middle
//		sensing_info.collided
//		sensing_info.speed
//		sensing_info.moving_forward
//		sensing_info.moving_angle
//		sensing_info.track_forward_angles
//		sensing_info.lap_progress
//		sensing_info.distance_to_way_points


		setSteering(sensingInfo, controls);
		setThrottle(sensingInfo, controls);
		setBrake(sensingInfo, controls);

		return controls;
	}

	class PathNode {
		float x;
		float y;
		float pivot;

		float cost;
		float first;

		public PathNode() {
			cost = Float.MAX_VALUE;
		}
	}

	private float getDistance(PathNode node1, PathNode node2) {
		float dx = node1.x - node2.x;
		float dy = node1.y - node2.y;
		return (float) Math.sqrt(dx * dx + dy * dy);
	}

	private void setSteering(CarStateValues sensingInfo, CarControls controls) {
		
		ArrayList<Float> trackForwardAngles = sensingInfo.track_forward_angles;
		final float roadWidth = 5;
		final float trackDistance = 10;

		ArrayList<ArrayList<PathNode>> nodes = new ArrayList<>();

		float currX = 0;
		float currY = 0;

		PathNode start = new PathNode();
		start.x = 0;
		start.y = 0;

		for (int i = 0; i < trackForwardAngles.size(); i++) {
			float trackAngleRadians = (float) Math.toRadians(trackForwardAngles.get(i)); // 라디안으로 한 번만 변환

			ArrayList<PathNode> layer = new ArrayList<>();

			for (float j = -roadWidth + 1; j < roadWidth; j++) {
				PathNode node = new PathNode();

				float distance = (float) Math.sqrt(trackDistance * trackDistance + j * j);
				float nodeAngleRadians = (float) Math.atan2(j, trackDistance);

				node.x = currX + distance * (float) Math.cos(trackAngleRadians + nodeAngleRadians);
				node.y = currY + distance * (float) Math.sin(trackAngleRadians + nodeAngleRadians);
				node.pivot = j;

				if (i == 0) {
					node.first = j;
					node.cost = getDistance(node, start);
				} else {
					float first = 0;
					float min = Float.MAX_VALUE;

					for (PathNode n : nodes.get(nodes.size() - 1)) {
						float cost = getDistance(node, n) + n.cost;

						if (cost < min) {
							min = cost;
							first = n.first;
						}
					}

					node.first = first;
					node.cost = min;
				}

				layer.add(node);
			}

			currX += trackDistance * (float) Math.cos(trackAngleRadians);
			currY += trackDistance * (float) Math.sin(trackAngleRadians);

			nodes.add(layer);
		}

		float middleAdjustment = ((nodes.get(19).get(3).first - sensingInfo.to_middle) / 40);

		float steerFactor = calculateSteerFactor(sensingInfo.speed);
		float setSteering = (getReferenceAngle(sensingInfo) - sensingInfo.moving_angle) / (steerFactor + 0.001f);
		setSteering += middleAdjustment;

		controls.setSteering(setSteering);

		System.out.println(nodes.get(19).get(3).first - sensingInfo.to_middle);
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

	private void setThrottle(CarStateValues sensingInfo, CarControls controls) {
		float throttleFactor = 0.9f / (Math.abs(getReferenceAngle(sensingInfo)) + 0.1f);
		throttleFactor = Math.min(throttleFactor, 0.11f);

		float setThrottle = 0.9f + throttleFactor;
		if (sensingInfo.speed < 60) {
			setThrottle = 1.0f;
		} else if (sensingInfo.speed > 150) {
			setThrottle = 0.97f;
		}

		controls.setThrottle(setThrottle);
	}
	
	private void setBrake(CarStateValues sensingInfo, CarControls controls) {

	}
}
