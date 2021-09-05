//(c) Chris Long 2012, see license.txt (BSD)

/**
   General-purpose utility functions
*/

#ifndef UTIL_H
#define UTIL_H

template <typename SRC, typename DEST>
DEST* copyArray(const int numItems, const SRC* source, DEST* dest = 0) {
    if (!dest) {
        dest = new DEST[numItems];
    }
    for (int i = 0; i < numItems; i++) {
        dest[i] = source[i];
    }
    return dest;
}

#endif
