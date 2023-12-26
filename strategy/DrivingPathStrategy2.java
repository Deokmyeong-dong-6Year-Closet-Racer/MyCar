package strategy;

import java.util.ArrayList;

import DrivingInterface.DrivingInterface.CarStateValues;
import pid.PIDController;
import strategy.NormalStrategy3.PathNode;

public class DrivingPathStrategy2 implements DrivingStrategy {

	float previousGoalSteer;
	private float previousGoalThrottle;
	boolean full_throttle = true;

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
		float angle;

		float cost;
		PathNode first;
		PathNode before;

		public PathNode() {
			cost = Float.MAX_VALUE;
		}
	}

	private float getAngleDifference(PathNode node1, PathNode node2) {
		return Math.abs(node1.angle - node2.angle);
	}

	public static float calculateAngleBetweenNodes(PathNode node2, PathNode node1) {
		float deltaX = node2.x - node1.x;
		float deltaY = node2.y - node1.y;

		float angleInRadians = (float) Math.atan2(deltaY, deltaX);
		float angleInDegrees = (float) Math.toDegrees(angleInRadians);

		return angleInDegrees;
	}

	// 대충 도로를 나눠서 점을찍고 점마다 가는비용을 다 계산해둠
	private ArrayList<ArrayList<PathNode>> createPathNodes(CarStateValues sensingInfo) {
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
					node.first = node;
					float angleDifference = getAngleDifference(node, start);
					node.cost = angleDifference * angleDifference;
					node.before = start;
				} else {
					PathNode first = node;
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

		return nodes;
	}

	private void setSteering(CarStateValues sensingInfo, CarControls controls) {
		ArrayList<ArrayList<PathNode>> nodes = createPathNodes(sensingInfo);

		PathNode ref_Node = nodes.get(19).get(3).first;
		float set_steering = (float) (Math.atan2(ref_Node.y - sensingInfo.to_middle, ref_Node.x)
				- Math.toRadians(sensingInfo.moving_angle));

		double skp = 0.7;
		double ski = 0.05;
		double skd = 0.1;
		PIDController pidController = new PIDController(skp, ski, skd);

		double tkp = 0.9;
		double tki = 0.1;
		double tkd = 0.1;
		PIDController throttlePidController = new PIDController(tkp, tki, tkd);
		float pidOut = (float) pidController.calculatePIDOutput(set_steering, previousGoalSteer);
		float ref_speed = (full_throttle && sensingInfo.speed < 120 ? 0
				: Math.abs(pidOut / 2) * (sensingInfo.speed / 200));
		
		previousGoalSteer = set_steering;
		set_steering = pidOut;

		float set_throttle = 1 - ref_speed;
		float set_brake = (float) (sensingInfo.speed > 100 && !full_throttle ? 0.2 : 0);

		float pdOut = (float) throttlePidController.calculatePDOutput(set_throttle, previousGoalThrottle);
		previousGoalThrottle = set_throttle;
		set_throttle = pdOut;

		controls.setThrottle(set_throttle);
		controls.setSteering(set_steering);
		controls.setBrake(set_brake);
	}

	private void setThrottle(CarStateValues sensingInfo, CarControls controls) {

	}

	private void setBrake(CarStateValues sensingInfo, CarControls controls) {
	}
}
