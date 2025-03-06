#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdlib>
#include "skeleton.h"
#include "motion.h"

void createGraphFile(Motion* orgMotion, Motion* firstMotion, Motion* secondMotion, int startFrame, int endFrame, int jointId, int axisId, const std::string& filename) {
    std::ofstream file(filename);

    if (!file) {
        printf("Error creating file!");
        exit(1);
    }

    file << "Frame," << "OY,Y1,Y2\n";

    for (int i = startFrame; i <= endFrame; i++) {
        Posture* orgPosture = orgMotion->GetPosture(i);
        Posture* firstPosture = firstMotion->GetPosture(i);
        Posture* secondPosture = secondMotion->GetPosture(i);

        double oy = orgPosture->bone_rotation[jointId].p[axisId];
        double y1 = firstPosture->bone_rotation[jointId].p[axisId];
        double y2 = secondPosture->bone_rotation[jointId].p[axisId];

        file << i << "," << oy << "," << y1 << "," << y2 << "\n";
    }

    file.close();

    printf("File created successfully");
}


int main(int argc, char** argv)
{
    if (argc < 9)
    {
        printf("Interpolates motion capture data.");
        printf("Usage: %s <input skeleton file> <First input motion capture file> <Second input motion capture file> <joint name> <Rotation Axis> <start frame> <end frame> <output file name>\n", argv[0]);
        printf("Rotation Axis : X or Y or Z\n");
        printf("Example: %s skeleton.asf motion1.amc motion2.amc root X 1 200 outputGraph\n", argv[0]);
        return -1;
    }

    char* inputSkeletonFile = argv[1];
    char* orgInputMotionCaptureFile = argv[2];
    char* firstInputMotionCaptureFile = argv[3];
    char* secondInputMotionCaptureFile = argv[4];
    char* jointName = argv[5];
    char* rotationAxisString = argv[6];
    char* startFrameString = argv[7];
    char* endFrameString = argv[8];
    char* outputFileNameString = argv[9];

    char* interpolation1 = nullptr;
    char* interpolation2 = nullptr;

    if (argc >= 11) {
        interpolation1 = argv[10];
        interpolation2 = argv[11];
    }
    
    Skeleton* pSkeleton = NULL;	// skeleton as read from an ASF file (input)
    Motion* pOrgInputMotion = NULL; // motion as read from an AMC file (input)
    Motion* pFirstInputMotion = NULL; // motion as read from an AMC file (input)
    Motion* pSecondInputMotion = NULL; // motion as read from an AMC file (input)

    printf("Loading skeleton from %s...\n", inputSkeletonFile);
    try
    {
        pSkeleton = new Skeleton(inputSkeletonFile, MOCAP_SCALE);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load skeleton from %s. Code: %d\n", inputSkeletonFile, exceptionCode);
        exit(1);
    }

    printf("Loading input motion from %s...\n", orgInputMotionCaptureFile);
    try
    {
        pOrgInputMotion = new Motion(orgInputMotionCaptureFile, MOCAP_SCALE, pSkeleton);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load motion from %s. Code: %d\n", orgInputMotionCaptureFile, exceptionCode);
        exit(1);
    }

    printf("Loading input motion from %s...\n", firstInputMotionCaptureFile);
    try
    {
        pFirstInputMotion = new Motion(firstInputMotionCaptureFile, MOCAP_SCALE, pSkeleton);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load motion from %s. Code: %d\n", firstInputMotionCaptureFile, exceptionCode);
        exit(1);
    }

    printf("Loading input motion from %s...\n", secondInputMotionCaptureFile);
    try
    {
        pSecondInputMotion = new Motion(secondInputMotionCaptureFile, MOCAP_SCALE, pSkeleton);
    }
    catch (int exceptionCode)
    {
        printf("Error: failed to load motion from %s. Code: %d\n", secondInputMotionCaptureFile, exceptionCode);
        exit(1);
    }

    int startFrame = strtol(startFrameString, NULL, 10);
    if (startFrame < 0 || startFrame > pFirstInputMotion->GetNumFrames() || startFrame > pSecondInputMotion->GetNumFrames())
    {
        printf("Error: invalid start Frame value (%d).\n", startFrame);
        exit(1);
    }
    printf("Start Frame=%d\n", startFrame);

    int endFrame = strtol(endFrameString, NULL, 10);
    if (endFrame < 0 || endFrame > pFirstInputMotion->GetNumFrames() || endFrame > pSecondInputMotion->GetNumFrames() || endFrame<=startFrame)
    {
        printf("Error: invalid end Frame value (%d).\n", endFrame);
        exit(1);
    }
    printf("End Frame=%d\n", endFrame);

    int jointId = pSkeleton->name2idx(jointName);
    printf("Joint Id=%d\n", jointId);


    int rotationAxis = 0;

    if (rotationAxisString[0] == 'X') {
        rotationAxis = 0;
    }
    else if (rotationAxisString[0] == 'Y') {
        rotationAxis = 1;
    }
    else if (rotationAxisString[0] == 'Z') {
        rotationAxis = 2;
    }
    printf("Rotation Axis=%d\n", rotationAxis);
   
    std::string outputCSVFileName = std::string(outputFileNameString) + ".csv";
    printf("Output File=%s\n", outputCSVFileName.c_str());

    std::string interpolationName1 = "Interpolation-1";
    std::string interpolationName2 = "Interpolation-2";

    if (interpolation1 != nullptr && interpolation2!=nullptr) {
        interpolationName1 = std::string(interpolation1);
        interpolationName2 = std::string(interpolation2);
    }
    
    createGraphFile(pOrgInputMotion, pFirstInputMotion, pSecondInputMotion, startFrame, endFrame, jointId, rotationAxis, outputCSVFileName);

    std::string command = "python plot_graph.py "+ outputCSVFileName + " "+ interpolationName1 + " "+interpolationName2 + " " + std::string(jointName) + " " + rotationAxisString[0];

    int result = system(command.c_str());

    if (result != 0) {
        printf("Error: Failed to execute Python script!");
    }

    return 0;
}
