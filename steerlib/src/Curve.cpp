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

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points

	if (!checkRobust())
		return;

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	Point start, end;
	start = controlPoints[0].position;
	float t = controlPoints[0].time;
	while (calculatePoint(end, t))
	{

		DrawLib::drawLine(start, end, curveColor, curveThickness);
		t = t + window;
		start = end;
	}

	end = controlPoints[controlPoints.size() - 1].position;
	DrawLib::drawLine(start, end, curveColor, curveThickness);

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

// Check Roboustness
bool Curve::checkRobust()
{
	if (type == hermiteCurve)
	{
		return (controlPoints.size() > 1);
		// for hermiteCurve, the number of control points should be not less than 2.
	}
	else if (type == catmullCurve)
	{
		return (controlPoints.size() > 3);
		// for catmullCurve, the number of control points should be not less than 3.
	}
	return false;
}

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

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	float p0 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
	float p1 = 3 * pow(normalTime, 2) - 2 * pow(normalTime, 3);
	float t0 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
	float t1 = pow(normalTime, 3) - pow(normalTime, 2);

	// Calculate position at t = time on Hermite curve
	newPosition = p0 * controlPoints[nextPoint - 1].position
		+ p1 * controlPoints[nextPoint].position
		+ t0 * controlPoints[nextPoint - 1].tangent*intervalTime
		+ t1 * controlPoints[nextPoint].tangent*intervalTime;

	// Return result
	return newPosition;
}


// Implement Catmull-Rom curve

// First, Calculate every controlpoint's tangent
Vector Curve::catmullCaculateTangent(const unsigned int index)
{
	Vector tangent, term1, term2;
	float coef1, coef2;

	if (index == 0)
	{
		coef1 = (controlPoints[2].time - controlPoints[0].time) / (controlPoints[2].time - controlPoints[1].time);
		term1 = (controlPoints[1].position - controlPoints[0].position) / (controlPoints[1].time - controlPoints[0].time);

		coef2 = (controlPoints[1].time - controlPoints[0].time) / (controlPoints[2].time - controlPoints[1].time);
		term2 = (controlPoints[2].position - controlPoints[0].position) / (controlPoints[2].time - controlPoints[0].time);

		tangent = coef1 * term1 - coef2 * term2;
		return tangent;
	}
	else if (index == controlPoints.size() - 1)
	{
		coef1 = (controlPoints[index].time - controlPoints[index - 2].time) / (controlPoints[index - 1].time - controlPoints[index - 2].time);
		term1 = (controlPoints[index].position - controlPoints[index - 1].position) / (controlPoints[index].time - controlPoints[index - 1].time);

		coef1 = (controlPoints[index].time - controlPoints[index - 1].time) / (controlPoints[index - 1].time - controlPoints[index - 2].time);
		term2 = (controlPoints[index].position - controlPoints[index - 2].position) / (controlPoints[index].time - controlPoints[index - 2].time);

		tangent = coef1*term1 - coef2*term2;
		return tangent;
	}
	else
	{
		coef1 = (controlPoints[index].time - controlPoints[index - 1].time) / (controlPoints[index + 1].time - controlPoints[index - 1].time);
		term1 = (controlPoints[index + 1].position - controlPoints[index].position) / (controlPoints[index + 1].time - controlPoints[index].time);

		coef2 = (controlPoints[index + 1].time - controlPoints[index].time) / (controlPoints[index + 1].time - controlPoints[index - 1].time);
		term2 = (controlPoints[index].position - controlPoints[index - 1].position) / (controlPoints[index].time - controlPoints[index - 1].time);

		tangent = coef1*term1 + coef2*term2;
		return tangent;
	}

}


Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	float normalTime, intervalTime;

	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	Vector tangent0 = catmullCaculateTangent(nextPoint - 1);
	Vector tangent1 = catmullCaculateTangent(nextPoint);

	float p0 = 2 * pow(normalTime, 3) - 3 * pow(normalTime, 2) + 1;
	float p1 = 3 * pow(normalTime, 2) - 2 * pow(normalTime, 3);
	float t0 = pow(normalTime, 3) - 2 * pow(normalTime, 2) + normalTime;
	float t1 = pow(normalTime, 3) - pow(normalTime, 2);

	newPosition = (p0 * controlPoints[nextPoint - 1].position) + (t0 * tangent0 * intervalTime) + (p1 * controlPoints[nextPoint].position) + (t1 * tangent1 * intervalTime);

	return newPosition;
}