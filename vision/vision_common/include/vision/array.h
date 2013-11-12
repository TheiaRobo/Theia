#ifndef VISION_COMMON_ARRAY
#define VISION_COMMON_ARRAY

#include <cstddef>
#include <vector>

template <class T>
class Array {
	protected:
		T array[];
		size_t arraySize;

	public:
		Array(size_t size);
		Array(std::vector<T> & vector);
		~Array();
		T & operator[](size_t index);
		size_t size();
		void sort(bool (* compFunc)(T & one, T & two));
};

#endif