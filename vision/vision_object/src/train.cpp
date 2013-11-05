#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include <vision/file.h>

#include "train.h"

using namespace std; 

void showFiles(vector<string> & fileVect){
	size_t numbFiles;
	numbFiles = fileVect.size();

	cout << endl;
	cout << "FILES (" << numbFiles << " total)" << endl;

	size_t i;
	for(i = 0; i < numbFiles; i++){
		cout << i << " " << fileVect[i] << endl;
	}

	cout << endl;
}

/**
* Check if a given file is a training file.
* If so the function returns true and the trainFile contains the infos.
*/
bool filterTrainFile(
	string path,
	TrainFile_t & trainFile
){
	/**
	* Find end of file name
	*/
	size_t lastDot;
	lastDot = path.find_last_of('.');

	size_t fileEndPos;
	switch(lastDot){
		case 0:
		case string::npos:
			return false;

		default:
			fileEndPos = lastDot - 1;
	}

	/**
	* Find start of file name
	* and end of directory name
	*/
	size_t lastDirSep;
	lastDirSep = path.find_last_of(VISION_DIR_SEPARATOR, fileEndPos);

	size_t fileStartPos;
	size_t dirEndPos;
	switch(lastDirSep){
		case 0:
		case string::npos:
			return false;

		default:
			fileStartPos = lastDirSep + 1;
			dirEndPos = lastDirSep - 1;
	}

	/**
	* Find start of directory name
	*/
	size_t secondLastDirSep;
	secondLastDirSep = path.find_last_of(VISION_DIR_SEPARATOR, dirEndPos);

	size_t dirStartPos;
	switch(secondLastDirSep){
		case string::npos:
			dirStartPos = 0;
			break;

		default:
			dirStartPos = secondLastDirSep + 1;
	}

	string dirName;
	dirName = path.substr(dirStartPos, dirEndPos - dirStartPos + 1);

	string fileName;
	fileName = path.substr(fileStartPos, fileEndPos - fileStartPos + 1);

	/**
	* Check if file name is a number
	*/
	int fileNameInt;
	stringstream strToIntStream(fileName);

	if(!(strToIntStream >> fileNameInt)){
		return false;
	}

	trainFile.path = path;
	trainFile.object = dirName;
	trainFile.rotation = fileNameInt;

	return true;
}

void showTrainFiles(vector<TrainFile_t> & trainFileVect){
	size_t numbFiles;
	numbFiles = trainFileVect.size();

	cout << endl;
	cout << "TRAINING FILES (" << numbFiles << " total)" << endl;

	size_t i;
	for(i = 0; i < numbFiles; i++){
		cout << i << endl;
		cout << " Path: " << trainFileVect[i].path << endl;
		cout << " Object: " << trainFileVect[i].object << endl;
		cout << " Rotation: " << trainFileVect[i].rotation << endl;
	}

	cout << endl;
}

void filterTrainFiles(
	vector<string> & fileVect,
	vector<TrainFile_t> & trainFileVect
){
	size_t numbFiles;
	numbFiles = fileVect.size();

	size_t i;
	for(i = 0; i < numbFiles; i++){
		bool isTrainFile;
		TrainFile_t trainFile;

		isTrainFile = filterTrainFile(fileVect[i], trainFile);

		if(isTrainFile){
			trainFileVect.push_back(trainFile);
		}
	}
}

/**
* Find and list training files
*/
int train(string trainDir){
	int errorCode;
	vector<string> fileVect;
	vector<TrainFile_t> trainFileVect;

	errorCode = visionFileScanDir(trainDir, fileVect);

	if(errorCode){
		cout << "Error: Could not scan training directory" << endl;
		return errorCode;
	}

	showFiles(fileVect);

	filterTrainFiles(fileVect, trainFileVect);
	showTrainFiles(trainFileVect);

	return 0;
}