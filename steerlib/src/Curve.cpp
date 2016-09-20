//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

//666666666666666666666666
// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return;

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	Point start, end;

	calculatePoint(start, 0);

	//start.y = start.y + 0.10;
	
	for (float t = controlPoints[0].time; t <= controlPoints[(controlPoints.size() - 1)].time; t += (float)(window)) {

		calculatePoint(end, t);
	//	end.y = end.y + 0.10;

		//draw
		DrawLib::drawLine(start, end, curveColor, curveThickness);
		start = end;

	}
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve
	
	return;
#endif
}

bool SortByTime(CurvePoint& p1, CurvePoint& p2)
{
	return p1.time < p2.time;
}

bool CheckEquals(CurvePoint& p1, CurvePoint& p2)
{
	return p1.time == p2.time;
}

//111111111111111111
// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	std::sort(controlPoints.begin(), controlPoints.end(), SortByTime);
	controlPoints.erase(unique(controlPoints.begin(), controlPoints.end(), CheckEquals), controlPoints.end());
	return;
}

//Do not change this function!!!!!!!!!!!!!!!!!!!!!!!!!
// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

//555555555555555
// Check Roboustness
bool Curve::checkRobust()
{
	if (controlPoints.size() < 2)
		return false;
	else
		return true;
}

//2222222222222222222222222222
// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for (int i = 0;i < controlPoints.size();i++)
	{
		if (time < controlPoints[i].time) {
			nextPoint = i;
			return true;
		}
	}
			return false;
}

//333333333333333333333
// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	float h1 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
	float h2 = 3 * pow(normalTime, 2) - 2 * pow(normalTime, 3);
	float h3 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
	float h4 = pow(normalTime, 3) - pow(normalTime, 2);

	// Calculate position at t = time on Hermite curve
	newPosition = h1*controlPoints[nextPoint - 1].position
		+ h2*controlPoints[nextPoint].position
		+ h3*controlPoints[nextPoint - 1].tangent*intervalTime
		+ h4*controlPoints[nextPoint].tangent*intervalTime;

	// Return result
	return newPosition;
}

//444444444444444444
// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{

	Point newPosition;

	float normalTime, intervalTime;

	Util::Vector s0, s1;

	CurvePoint currentPt = controlPoints[nextPoint - 1];

	CurvePoint nextPt = controlPoints[nextPoint];



	// Calculate time interval, and normal time required for later curve calculations

	intervalTime = nextPt.time - currentPt.time;

	normalTime = (time - currentPt.time) / intervalTime;



	// Calculate position at t = time on Catmull-Rom curve

	if (nextPoint == 1) {

		//calculate s0 with special case

		CurvePoint otherPt = controlPoints[nextPoint + 1];

		s0 = (otherPt.time - currentPt.time) / (otherPt.time - nextPt.time)*(nextPt.position - currentPt.position) / (nextPt.time - currentPt.time) -

			(nextPt.time - currentPt.time) / (otherPt.time - nextPt.time)*(otherPt.position - currentPt.position) / (otherPt.time - currentPt.time);

	}

	else {

		//calculate as normal

		CurvePoint otherPt = controlPoints[nextPoint - 2];

		s0 = (currentPt.time - otherPt.time) / (nextPt.time - otherPt.time)*(nextPt.position - currentPt.position) / (nextPt.time - currentPt.time) +

			(nextPt.time - currentPt.time) / (nextPt.time - otherPt.time)*(currentPt.position - otherPt.position) / (currentPt.time - otherPt.time);

	}

	if (nextPoint == (controlPoints.size() - 1)) {

		//calculate s1 with special case

		CurvePoint otherPt = controlPoints[nextPoint - 2];

		s1 = (nextPt.time - otherPt.time) / (currentPt.time - otherPt.time)*(nextPt.position - currentPt.position) / (nextPt.time - currentPt.time) -

			(nextPt.time - currentPt.time) / (currentPt.time - otherPt.time)*(nextPt.position - otherPt.position) / (nextPt.time - otherPt.time);

	}

	else {

		//calculate as normal

		CurvePoint otherPt = controlPoints[nextPoint + 1];

		s1 = (nextPt.time - currentPt.time) / (otherPt.time - currentPt.time)*(otherPt.position - nextPt.position) / (otherPt.time - nextPt.time) +

			(otherPt.time - nextPt.time) / (otherPt.time - currentPt.time)*(nextPt.position - currentPt.position) / (nextPt.time - currentPt.time);

	}

	// Calculate position at t = time on Catmull-Rom curve

	float h1 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
	float h2 = 3 * pow(normalTime, 2) - 2 * pow(normalTime, 3);
	float h3 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
	float h4 = pow(normalTime, 3) - pow(normalTime, 2);

	newPosition = h1*currentPt.position + h3*s0*intervalTime + h2*nextPt.position + h4*s1*intervalTime;

	// Return result
	return newPosition;
}