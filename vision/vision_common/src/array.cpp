#include <algorithm>
#include <cstddef>
#include <vector>
#include <vision/array.h>

template <class T>
Array<T>::Array(){
	arraySize = 0;
	array = NULL;
}

template <class T>
Array<T>::Array(size_t size){
	if(arraySize > 0){
		arraySize = size;
		array = new T[arraySize];
	}else{
		arraySize = 0;
		array = NULL;
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
	if(!array) return;

	delete[] array;
}

template <class T>
T & Array<T>::operator[](size_t index){
	if(!array) return T();
	if(index < 0) return T();
	if(index >= arraySize) return T();
	return array[index];
}

template <class T>
size_t Array<T>::size(){
	return arraySize;
}

template <class T>
void Array<T>::sort(bool (* compFunc)(T & one, T & two)){
	if(!array) return;

	std::sort(array, array + arraySize, compFunc);
}