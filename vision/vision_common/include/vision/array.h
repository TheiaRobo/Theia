#ifndef VISION_COMMON_ARRAY
#define VISION_COMMON_ARRAY

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

template <class T>
class Array {
	protected:
		T * array;
		size_t arraySize;

	public:
		Array();
		Array(size_t size);
		Array(std::vector<T> & vector);
		~Array();
		T & operator[](size_t index);
		size_t size();
		void sort();
		void sort(bool (* compFunc)(T & one, T & two));
};

template <class T>
Array<T>::Array(){
	arraySize = 0;
	array = new T[0];
}

template <class T>
Array<T>::Array(size_t size){
	if(arraySize > 0){
		arraySize = size;
		array = new T[arraySize];
	}else{
		arraySize = 0;
		array = new T[0];
	}
}

template <class T>
Array<T>::Array(std::vector<T> & vector){
	arraySize = vector.size();
	array = new T[arraySize];

	size_t i;
	for(i = 0; i < arraySize; i++){
		array[i] = vector[i];
	}
}

template <class T>
Array<T>::~Array(){
	delete[] array;
}

template <class T>
T & Array<T>::operator[](size_t index){
	if(index < 0 || index >= arraySize){
		throw std::out_of_range("Array index of out range");
	}

	return array[index];
}

template <class T>
size_t Array<T>::size(){
	return arraySize;
}

template <class T>
void Array<T>::sort(){
	std::sort(array, array + arraySize);
}

template <class T>
void Array<T>::sort(bool (* compFunc)(T & one, T & two)){
	std::sort(array, array + arraySize, compFunc);
}

#endif