package strategy;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import DrivingInterface.DrivingInterface.CarStateValues;
import DrivingInterface.DrivingInterface.ObstaclesInfo;
import pid.PIDController;
import strategy.DrivingPathStrategy3.PathNode;

public class DrivingPathStrategy4 implements DrivingStrategy {

	float previousGoalSteer;
	boolean full_throttle = true;
	float frontCost;

	ArrayList<ArrayList<PathNode>> nodes;
	ArrayList<PathNode> DrivingPath;

	float refSpeed;

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

		createPathNodes(sensingInfo);

		frontCost = DrivingPath.get(10).cost;
		// System.out.println(frontCost);

		if (frontCost < 150)
			refSpeed = 200;
		else if (frontCost < 300)
			refSpeed = 130;
		else
			refSpeed = 100;

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

	private void createPathNodes(CarStateValues sensingInfo) {
		ArrayList<Float> trackForwardAngles = sensingInfo.track_forward_angles;
		final float roadWidth = sensingInfo.half_road_limit - 1.25f;
		final float trackDistance = 10;

		nodes = new ArrayList<>();

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

			for (float j = -roadWidth + 2.5f; j < roadWidth - 2.5f; j++) {
				PathNode node = new PathNode();

				float distance = (float) Math.sqrt(trackDistance * trackDistance + j * j);
				float nodeAngleRadians = (float) Math.atan2(j, trackDistance);

				node.x = currX + distance * (float) Math.cos(trackAngleRadians + nodeAngleRadians);
				node.y = currY + distance * (float) Math.sin(trackAngleRadians + nodeAngleRadians);
				node.pivot = j;

				if (!checkObstacles(sensingInfo, node))
					continue;

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

		PathNode minCostNode = nodes.get(10).get(nodes.get(19).size() / 2);
		DrivingPath = new ArrayList<PathNode>();
		System.out.println("????????????????????");
		while (minCostNode != null) {
			System.out.println("(" + minCostNode.x + ", " + minCostNode.y + "),");
			DrivingPath.add(minCostNode);
			minCostNode = minCostNode.before;
		}

		Collections.reverse(DrivingPath);
	}

	private boolean checkObstacles(CarStateValues sensingInfo, PathNode node) {
		float nodeDistance = nodes.size() * 10;
		float nodeToMiddle = node.pivot;
		
		for (ObstaclesInfo obstacles : sensingInfo.track_forward_obstacles) {
			if (((obstacles.dist + 2) > nodeDistance) && ((obstacles.dist - 2) < nodeDistance)
					&& ((obstacles.to_middle - 2) < nodeToMiddle) && ((obstacles.to_middle + 2) > nodeToMiddle))
				return false;
		}

		return true;
	}

	private void setSteering(CarStateValues sensingInfo, CarControls controls) {

		int nodeNum = (int) (sensingInfo.speed / 50) + 1;

		PathNode ref_Node = new PathNode();

		ref_Node.x = 0;
		ref_Node.y = 0;

		for (int i = 0; i < nodeNum; i++) {
			ref_Node.x += DrivingPath.get(i).x;
			ref_Node.y += DrivingPath.get(i).y;
		}

		ref_Node.x /= nodeNum;
		ref_Node.y /= nodeNum;

		float set_steering = (float) (Math.atan2(ref_Node.y - sensingInfo.to_middle, ref_Node.x)
				- Math.toRadians(sensingInfo.moving_angle));

		double skp = 0.7;
		double ski = 0.05;
		double skd = 0.1;
		PIDController pidController = new PIDController(skp, ski, skd);

		float pidOut = (float) pidController.calculatePIDOutput(set_steering, previousGoalSteer);

		previousGoalSteer = set_steering;
		set_steering = pidOut;

		controls.setSteering(set_steering);
	}

	private void setThrottle(CarStateValues sensingInfo, CarControls controls) {

		float buff = sensingInfo.speed - refSpeed;

		if (buff < 0)
			controls.setThrottle(1f);
		else
			controls.setThrottle(0.6f);

	}

	private void setBrake(CarStateValues sensingInfo, CarControls controls) {

		float buff = sensingInfo.speed - refSpeed;

		if (buff > 0) {
			if (buff > 50)
				controls.setBrake(0.5f);
			else if (buff > 30)
				controls.setBrake(0.2f);
			else if (buff > 10)
				controls.setBrake(0.1f);
		} else {
			controls.setBrake(0f);
		}
	}
}
