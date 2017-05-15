# multi-object-tracker
Multiple objects tracker using openCV and dlib
![Alt text](/example.png)

# Requsites
__1. openCV 3.0__  
__2. dlib-19.3__  
__3. visual studio 15 (c+11 is necessary)__  

# How to compile
To run this program, you need to compile __opencv__ and __dlib__. You will be able to compile __opencv__ easily from many websites.
The folliwing instruction is how to compile dlib. If you have troubles in compiling dlib then you can just use __tracker/vs_solution/MultiObjectTracker.sln__ instead, but you need to change opencv path with your environment.  

1. Make new visual studio project.  

2. Include __dlib-19.3/dlib/all/source.cpp__ and __tracker/src/Tracker.cpp__ to your project. Tracker.cpp file will be divided into several files soon.

3. Make new folder(filter) in your project. Include all files(all cpp files and headers) in __dlib-19.3/dlib/external/libjpeg__ to your project.

4. Open __project properties -> Configuration Properties -> C/C++__. You can see __additional include directory__. Write __dlib-19.3__ (next directory should be dlib) to there.

5. Opem __project properties -> Configuration Properties -> C/C++ -> Preprocessor__. You can see __Preprocessor Definitions__. Write __DLIB_JPEG_STATIC__ and __DLIB_JPEG_SUPPORT__ to there.

6. Build.

# How to run 

# Future updates

1.  
