/*
 * pal.inl
 *
 *  Created on: Apr 5, 2014
 *      Author: david
 */

#ifndef PAL_INL_
#define PAL_INL_

#include <pal.h>
#include <sstream>
#include <iostream>

template<typename T>
T palPhysics::GetInitProperty( PAL_STRING name, T defaultVal, T min, T max )
{
	T result = defaultVal;
	std::istringstream ss;
	if (!m_Properties[name].empty()) {
		ss.str(m_Properties[name]);
		ss >> result;
		if (result < min) result = min;
		else if (result > max) result = max;
	}
	return result;
}


#endif /* PAL_INL_ */
