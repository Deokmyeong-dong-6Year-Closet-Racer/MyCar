package strategy;

import java.util.ArrayList;

import DrivingInterface.DrivingInterface.CarStateValues;

public class DrivingPathStrategy2 implements DrivingStrategy {

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
		float angle; // 각도를 저장할 필드

		float cost;
		float first;
		PathNode before;

		public PathNode() {
			cost = Float.MAX_VALUE;
		}
	}

	private float getAngleDifference(PathNode node1, PathNode node2) {
		float difference = node1.angle - node2.angle;
		return Math.abs(difference);
	}

	public static float calculateAngleBetweenNodes(PathNode node2, PathNode node1) {
		float deltaX = node2.x - node1.x;
		float deltaY = node2.y - node1.y;

		float angleInRadians = (float) Math.atan2(deltaY, deltaX);
		float angleInDegrees = (float) Math.toDegrees(angleInRadians);

		return angleInDegrees;
	}

	private void setSteering(CarStateValues sensingInfo, CarControls controls) {
		ArrayList<Float> trackForwardAngles = sensingInfo.track_forward_angles;
		final float roadWidth = sensingInfo.half_road_limit - 1.25f;
		final float trackDistance = 10;

		ArrayList<ArrayList<PathNode>> nodes = new ArrayList<>();

		float currX = 0;
		float currY = 0;

		PathNode start = new PathNode();
		start.x = 0;
		start.y = sensingInfo.to_middle;
		start.angle = 0;

		System.out.println("==========");

		for (int i = 0; i < trackForwardAngles.size(); i++) {
			float trackAngleRadians = (float) Math.toRadians(trackForwardAngles.get(i));
			ArrayList<PathNode> layer = new ArrayList<>();

			for (float j = -roadWidth + 1; j < roadWidth; j += 2) {
				PathNode node = new PathNode();

				float distance = (float) Math.sqrt(trackDistance * trackDistance + j * j);
				float nodeAngleRadians = (float) Math.atan2(j, trackDistance);

				node.x = currX + distance * (float) Math.cos(trackAngleRadians + nodeAngleRadians);
				node.y = currY + distance * (float) Math.sin(trackAngleRadians + nodeAngleRadians);
				node.pivot = j;

				if (i == 0) {
					node.angle = calculateAngleBetweenNodes(node, start);
					node.first = j;
					float angleDifference = getAngleDifference(node, start);
					node.cost = angleDifference * angleDifference;
					node.before = start;
				} else {
					float first = 0;
					float minCost = Float.MAX_VALUE;
					float minAngle = Float.MAX_VALUE;
					PathNode bestPreviousNode = null;

					for (PathNode previousNode : nodes.get(i - 1)) {
						node.angle = calculateAngleBetweenNodes(node, previousNode);
						float angleDifference = getAngleDifference(node, previousNode);
						float cost = angleDifference * angleDifference + previousNode.cost;

						if (cost < minCost) {
							first = previousNode.first;
							minCost = cost;
							minAngle = node.angle;
							bestPreviousNode = previousNode;
						}
					}

					node.first = first;
					node.cost = minCost;
					node.angle = minAngle;
					node.before = bestPreviousNode;
				}

				System.out.println("(" + node.x + ", " + node.y + "),");

				layer.add(node);
			}

			currX += trackDistance * (float) Math.cos(trackAngleRadians);
			currY += trackDistance * (float) Math.sin(trackAngleRadians);

			nodes.add(layer);
		}

		PathNode minCostNode = nodes.get(19).get(2);
		System.out.println("????????????????????");
		while (minCostNode != null) {
			System.out.println("(" + minCostNode.x + ", " + minCostNode.y + "),");
			minCostNode = minCostNode.before;
		}

		controls.setSteering(0);
	}

	private void setThrottle(CarStateValues sensingInfo, CarControls controls) {
		controls.setThrottle(0.5f);
	}

	private void setBrake(CarStateValues sensingInfo, CarControls controls) {
		controls.setBrake(0);
	}
}
