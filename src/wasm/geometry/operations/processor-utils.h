/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */


#pragma once

bool IsCCW(IfcCurve &curve) const
{
	double sum = 0;

	for (size_t i = 0; i < curve.size(); i++)
	{
		glm::dvec3 pt1 = curve.at((i - 1) % curve.size());
		glm::dvec3 pt2 = curve.at(i);

		sum += (pt2.x - pt1.x) * (pt2.y + pt1.y);
	}

	return sum < 0;
}