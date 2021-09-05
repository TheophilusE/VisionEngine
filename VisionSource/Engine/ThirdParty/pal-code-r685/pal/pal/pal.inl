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
T palPhysics::GetInitProperty( const PAL_STRING& name, T defaultVal, T min, T max ) const
{
	T result = defaultVal;
	std::istringstream ss;
	PropertyMap::const_iterator i = m_Properties.find(name);
	if (i != m_Properties.end()) {
		ss.str(i->second);
		ss >> result;
		if (result < min) result = min;
		else if (result > max) result = max;
	}
	return result;
}


#endif /* PAL_INL_ */
