#include <cstddef>
#include <vector>
#include <vision/array.h>

using namespace std;

template <class T>
Array<T>::Array(size_t size){
	arraySize = size;
	array = new T[arraySize];
}

template <class T>
Array<T>::Array(vector<T> & vector){
	arraySize = vector.size();
	array = new T[arraySize];

	size_t i;
	for(i = 0; i < arraySize; i++){
		array[i] = vector[i];
	}
}

template <class T>
size_t Array<T>::size(){
	return arraySize;
}

template <class T>
T & Array<T>::operator[](size_t index){
	if(index < 0) return T();
	if(index >= arraySize) return T();
	return array[index];
}