/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.  */


#pragma once

inline double VectorToAngle(double x, double y)
{
	double dd = sqrt(x * x + y * y);
	double xx = x / dd;
	double yy = y / dd;

	double angle = asin(xx);
	double cosv = cos(angle);

	if (glm::abs(yy - cosv) > 1e-5)
	{
		angle = acos(yy);
		double sinv = sin(angle);
		cosv = cos(angle);
		if (glm::abs(yy - cosv) > 1e-5 || glm::abs(xx - sinv) > 1e-5)
		{
			angle = angle + (CONST_PI - angle) * 2;
			sinv = sin(angle);
			cosv = cos(angle);
			if (glm::abs(yy - cosv) > 1e-5 || glm::abs(xx - sinv) > 1e-5)
			{
				angle = angle + CONST_PI;
			}
		}
	}

	return (angle / (2 * CONST_PI)) * 360;
}