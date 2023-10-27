#ifndef FORWARD_H
#define FORWARD_H

//  simple geometric approach assuming zero twists and offsets, neglecting ee orientation
#include <iostream>
#include <cmath>

struct LinkLengths {
	double a1;
	double a2;
	double a3;
};

struct EndEffectorPosition {
	double x;
	double y;
	double z;
};

struct JointAngles {
	double theta1;
	double theta2;
	double theta3;
};

EndEffectorPosition calculateForwardKinematics(const LinkLengths& linkLengths, const JointAngles& angles) {
	EndEffectorPosition position;

	double theta1 = angles.theta1;
	double theta2 = angles.theta2;
	double theta3 = angles.theta3;

	// end effector position in the xy plane
	double x = linkLengths.a1 * cos(theta1) + linkLengths.a2 * cos(theta1 + theta2);
	double y = linkLengths.a1 * sin(theta1) + linkLengths.a2 * sin(theta1 + theta2);

	// end effector position in the z axis
	double z = linkLengths.a1 * sin(theta3) + linkLengths.a2 * sin(theta2 - theta3);

	position.x = x;
	position.y = y;
	position.z = z;

	return position;
}

#endif //FORWARD_H