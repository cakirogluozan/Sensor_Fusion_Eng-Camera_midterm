## Sensor Fusion Engineering Nanodegree

### Camera Midterm Project

#### Milestones

* MP.1 Data Buffer Optimization  
        
        MidTermProject_Camera_Student.cpp line:74-82

    In order to optimize data buffer, the algorithm reads an image, and if the buffer size is less than 2, it appends; else, it deletes the first image and appends the new one.

* MP.2 Keypoint Detection        

        MidTermProject_Camera_Student.cpp line:74-82
        matching2D_Student.cpp            line:102-237
    
    Keypoint detection has been implemented by the listed detectors: "SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT" by defining functions for "SHITOMASI", "HARRIS" and the rest (modern).

* MP.3 Keypoint Removal
        
        MidTermProject_Camera_Student.cpp line:116-135

    In order to focus on the keypoints on the preceeding car, the keypoints outside of the predefined bounding box filtered out.

* MP.4 Keypoint Descriptors      

        MidTermProject_Camera_Student.cpp line:164
        matching2D_Student.cpp            line:64-98

    Keypoint descriptors has been implemented by the listed descriptors: "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT".

* MP.5 Descriptor Matching 
        
        MidTermProject_Camera_Student.cpp line:199 
        matching2D_Student.cpp            line:26-34
    
    Descriptor matching has been implemented in order to check keypoint match in two side.

* MP.6 Descriptor Distance Ratio 

        matching2D_Student.cpp            line:42-59

    FLANN matching has been added. KNN matching implemented and descriptor distance ratio filtering with t=0.8 performed.

* MP.7-8-9 Performance Evaluation 1-2-3  

        LOG.txt
    
    MP.7, MP.8 and MP9 are logged in LOG.txt file. 

    * Best performance by considering time consumption of "SHITOMASI" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            SHITOMASI BRIEF - time detection     : 0.0801808 
            SHITOMASI BRIEF - time extraction    : 0.00593766
            SHITOMASI BRIEF - number kpts        : 1062
            SHITOMASI BRIEF - number matched kpts: 953


    * Best performance by considering time consumption of "HARRIS" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            HARRIS ORB - time detection     : 0.0836451
            HARRIS ORB - time extraction    : 0.0216408
            HARRIS ORB - number kpts        : 166
            HARRIS ORB - number matched kpts: 146

    * Best performance by considering time consumption of "FAST" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            FAST SIFT - time detection     : 0.015786
            FAST SIFT - time extraction    : 0.132676
            FAST SIFT - number kpts        : 3716
            FAST SIFT - number matched kpts: 2805

    * Best performance by considering time consumption of "BRISK" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            BRISK BRIEF - time detection     : 1.85614
            BRISK BRIEF - time extraction    : 0.00627846
            BRISK BRIEF - number kpts        : 2460
            BRISK BRIEF - number matched kpts: 1676

    * Best performance by considering time consumption of "ORB" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            ORB ORB - time detection     : 0.160082
            ORB ORB - time extraction    : 0.0801741
            ORB ORB - number kpts        : 1059
            ORB ORB - number matched kpts: 754

    
    * Best performance by considering time consumption of "AKAZE" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            AKAZE BRIEF - time detection     : 0.341429
            AKAZE BRIEF - time extraction    : 0.00557768
            AKAZE BRIEF - number kpts        : 1493
            AKAZE BRIEF - number matched kpts: 1257

    
    * Best performance by considering time consumption of "SIFT" detector and "BRISK", "BRIEF", "ORB", "FREAK", "AKAZE", "SIFT" descriptor combination.

            SIFT BRIEF - time detection     : 0.5925
            SIFT BRIEF - time extraction    : 0.00636176
            SIFT BRIEF - number kpts        : 1234
            SIFT BRIEF - number matched kpts: 693

    In this project, I would take advantage of AKAZE and BRISK detector-descriptor combination. While detection time lasts around 0.34 ms for 10 images, extraction time is relatively slow than the other combinations, as well as it has the best performance rate (1257/1493) which indicates highest accuracy while detecting more than enough keypoints.