// C
#include <dirent.h>
#include <sys/types.h>

// C ++
#include <iostream>
#include <string>
#include <vector>

// ROS
#include <vision/file.h>

typedef struct dirent DIRENT;

using namespace std;

int visionFileScanDir(string dirName, vector<string> & fileVect){
    DIR * dirPtr;
    struct dirent * dirEntPtr;
    vector<string> dirVect;

    dirPtr = opendir(dirName.c_str());

    if(!dirPtr){
        cout << "Error: Could not open directory" << endl;
        return -1;
    }

    /**
    * Find regular files and subdirectories
    */
    do{
        dirEntPtr = readdir(dirPtr);

        // are we done here?
        if(!dirEntPtr){
            break;
        }

        string fileName;
        fileName = string(dirEntPtr->d_name);

        // ignore recursive file names
        if(fileName == "." || fileName == ".."){
            continue;
        }

        string fullFileName;
        fullFileName = dirName + '/' + fileName;

        switch(dirEntPtr->d_type){
            // regular file
            case DT_REG:
                fileVect.push_back(fullFileName);
                break;

            // directory
            case DT_DIR:
                dirVect.push_back(fullFileName);
                break;
        }
    }while(true);

    closedir(dirPtr);

    /**
    * Scan subdirectories  
    */
    size_t numbSubDirs;
    numbSubDirs = dirVect.size();

    int errorCode;
    for(size_t i = 0; i < numbSubDirs; i++){
        errorCode = visionFileScanDir(dirVect[i], fileVect);

        if(errorCode){
            cout << "Error: Could not read subdirectory" << endl;
            return errorCode;
        }
    }

    return 0;
}

int visionFileFilterTraining(
    vector<string> & fileVect,
    vector<VisionTrainingFile_t> & trainingFileVect
){
  return 0;  
}