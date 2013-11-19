#include <cerrno>
#include <cstdio>
#include <dirent.h>
#include <iostream>
#include <sys/types.h>
#include <vision/file.h>

using namespace std;

typedef struct dirent DIRENT;

int visionFileFindDirs(
    const string inPath,
    vector<string> & outDirVect
){
    int errorCode;

    outDirVect.clear();

    DIR * dirPtr = opendir(inPath.c_str());
    if(!dirPtr){
        cout << "Error in visionFileFindDirs" << endl;
        cout << "Could not open directory" << endl;
        return errno;
    }

    do{
        DIRENT * dirEntPtr = readdir(dirPtr);
        if(!dirEntPtr) break;
        if(!dirEntPtr->d_type == DT_DIR) continue;
        
        string fileName = string(dirEntPtr->d_name);
        outDirVect.push_back(fileName);
    }while(true);

    errorCode = closedir(dirPtr);
    if(errorCode){
        cout << "Error in visionFileFindDirs" << endl;
        cout << "Could not close directory" << endl;
        return errno;
    }

    return errorCode;
}

bool visionFileExists(const string inPath){
    FILE * filePtr = fopen(inPath.c_str(), "r");
    if(filePtr){
        fclose(filePtr);
        return true;
    }else{
        return false;
    }
}