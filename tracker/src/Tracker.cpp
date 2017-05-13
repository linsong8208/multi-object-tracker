#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <dlib/image_processing.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/dir_nav.h>
#include <dlib/opencv.h>

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <string>
#include <io.h>
#include <cstdlib>

#define FAIL  -1
#define SUCCESS     1
#define FALSE  0
#define TRUE  1

#define OUT_OF_FRAME    2

#define FRAME_READER_IS_MADE  3
#define CANNOT_OPEN_DIR    4
#define WAITING_FOR_READING_FRAME 5
#define READING_FRAME_STARTED  6
#define READING_FRAME_DONE   7

#define ENTER  13
#define ESC   27

using namespace std;


/* ==========================================================================

Class : Util

Many useful but not fundamental functions are implemented in this class.
All functions are static functions so don't need to make class Util object
to use these functions.

========================================================================== */
class Util
{
public:

	/* --------------------------------------------
	Function : cvtRectToRect
	Convert cv::Rect to dlib::drectangle
	----------------------------------------------- */
	static dlib::drectangle cvtRectToDrect(cv::Rect _rect)
	{
		return dlib::drectangle(_rect.tl().x, _rect.tl().y, _rect.br().x - 1, _rect.br().y - 1);
	}


	/* -------------------------------------------------
	Function : cvtMatToArray2d
	convert cv::Mat to dlib::array2d<unsigned char>
	------------------------------------------------- */
	static dlib::array2d<unsigned char> cvtMatToArray2d(cv::Mat _mat) // cv::Mat, not cv::Mat&. Make sure use copy of image, not the original one when converting to grayscale
	{

		//Don't need to use color image in HOG-feature-based tracker
		//Convert color image to grayscale
		if (_mat.channels() == 3)
			cv::cvtColor(_mat, _mat, cv::COLOR_RGB2GRAY);

		//Convert opencv 'MAT' to dlib 'array2d<unsigned char>'
		dlib::array2d<unsigned char> dlib_img;
		dlib::assign_image(dlib_img, dlib::cv_image<unsigned char>(_mat));

		return dlib_img;
	}


	/* -----------------------------------------------------------------
	Function : setRectToImage
	Put all tracking results(new rectangle) on the frame image
	Parameter _rects is stl container(such as vector..) filled with
	cv::Rect
	----------------------------------------------------------------- */
	template <typename Container>
	static void setRectToImage(cv::Mat& _mat_img, Container _rects)
	{
		std::for_each(_rects.begin(), _rects.end(), [&_mat_img](cv::Rect rect) {
			cv::rectangle(_mat_img, rect, cv::Scalar(0, 0, 255));
		});
	}
};

/* ==========================================================================

Class : TargetRectDrawer

This class makes users draw initial rectangles on the window.
It can draw many rectangles and these rectangles are used to initialize
the SingleTracker class.

========================================================================== */
class TargetRectDrawer
{
private:
	cv::Mat img_orig;  // Original Image
	cv::Mat img_prev;  // Previous image(before draw new rectangle)
	cv::Mat img_draw;  // Draw rectangle here
	std::vector<std::pair<cv::Rect, cv::Scalar>> rect_vec;  // Drawing result

	int start_x;   // the first x (x when left mouse button is clicked)
	int start_y;   // the first y (y when left mouse button is clicked)

	bool is_drawing_rect; // is drawing rectangle?

public:
	TargetRectDrawer() : start_x(0), start_y(0), is_drawing_rect(false) {}

	/* Set Function */
	void setImgOrig(cv::Mat& _img) { this->img_orig = _img.clone(); } // Deep copy
	void setImgPrev(cv::Mat& _img) { this->img_prev = _img.clone(); } // Deep copy
	void setImgDraw(cv::Mat& _img) { this->img_draw = _img.clone(); } // Deep copy
	void setStartX(int _start_x) { this->start_x = _start_x; }
	void setStartY(int _start_y) { this->start_y = _start_y; }
	void setIsDrawingRect(bool _is_drawing) { this->is_drawing_rect = _is_drawing; }

	/* Get Function */
	cv::Mat& getImgOrig() { return this->img_orig; }
	cv::Mat& getImgPrev() { return this->img_prev; }
	cv::Mat& getImgDraw() { return this->img_draw; }
	bool getIsDrawingRect() { return this->is_drawing_rect; }
	std::vector<std::pair<cv::Rect, cv::Scalar>>& getRectVec() { return this->rect_vec; }

	/* Core Function */
	int initTargetRectDrawer(cv::Mat& _img);
	static void wrapperCallBackFunc(int event, int x, int y, int flags, void* userdata);
	void CallBackFunc(int event, int x, int y, int flags);
	std::vector<std::pair<cv::Rect, cv::Scalar>>& drawInitRect(cv::Mat& _mat_img);
};

/* -----------------------------------------------------------------
Function : initTargetRectDrawer

Initialize key variables in TargetRectDrawer class.
----------------------------------------------------------------- */
int TargetRectDrawer::initTargetRectDrawer(cv::Mat& _img)
{
	if (_img.empty())
	{
		std::cout << "====================== Error Occured! =======================" << std::endl;
		std::cout << "Function : int TargetRedcDrawer::initTargetRectDrawer" << std::endl;
		std::cout << "Parameter cv::Mat& _img is empty image!" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	this->img_orig = _img.clone(); // deep copy
	this->img_prev = _img.clone(); // deep copy
	this->img_draw = _img.clone(); // deep copy

	return SUCCESS;
}

/* -----------------------------------------------------------------
Function : callBackFunc

Callback fuction for mouse event. Implement proper action for
each expected mouse event drawing rectangle like left button down,
move, left button up.
----------------------------------------------------------------- */
void TargetRectDrawer::CallBackFunc(int event, int x, int y, int flags)
{
	// When left mouse button is clicked (Normally, next event is moving mouse, EVENT_MOUSEMOVE)
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		// Set is_drawing_rect true
		this->setIsDrawingRect(true);

		cv::Rect new_rect(x, y, 0, 0);
		cv::Scalar new_color(rand() % 256, rand() % 256, rand() % 256);
		std::pair<cv::Rect, cv::Scalar> new_pair = std::make_pair(new_rect, new_color);

		// push new result to the vector
		this->getRectVec().push_back(new_pair);

		// Set start_x and start_y
		this->setStartX(x);
		this->setStartY(y);
	}
	// when mouse is moving (Normally, after left mouse button click, EVENT_LBUTTONDOWN)
	else if (event == cv::EVENT_MOUSEMOVE)
	{
		// when left mouse button is clicked at the same time
		if (getIsDrawingRect())
		{
			int new_width = x - this->start_x;
			int new_height = y - this->start_y;

			if ((new_width * new_height) > 0)
			{
				this->getRectVec().back().first.width = new_width;
				this->getRectVec().back().first.height = new_height;
			}
			else if ((new_width * new_height) < 0)
			{
				this->getRectVec().back().first.x = x; //?
				this->getRectVec().back().first.width = -new_width;  //?
				this->getRectVec().back().first.height = new_height; //?
			}

			this->img_draw = this->img_prev.clone();

			cv::Rect new_rect = this->getRectVec().back().first;
			cv::Scalar new_color = this->getRectVec().back().second;

			cv::rectangle(this->img_draw, new_rect, new_color, 2);
			cv::imshow("Tracking System", this->img_draw);
		}
	}
	// when left mouse button is up (Normally, after moving mouse, EVENT_MOUSEMOVE)
	else if (event == cv::EVENT_LBUTTONUP)
	{
		setIsDrawingRect(false);

		cv::Rect& new_rect = this->getRectVec().back().first;  // Want to change some values in new_rect.
		cv::Scalar new_color = this->getRectVec().back().second;

		if (new_rect.width < 0)
		{
			new_rect.x += new_rect.width;
			new_rect.width = -new_rect.width;
		}

		if (new_rect.height < 0)
		{
			new_rect.y += new_rect.height;
			new_rect.height = -new_rect.height;
		}

		cv::rectangle(this->img_draw, new_rect, new_color, 2);

		cv::imshow("Tracking System", this->img_draw);

		// Drawing is done. Make current image as previous image
		this->img_prev = img_draw.clone();
	}
}

/* -----------------------------------------------------------------

Function : WrapperCallBackFunc

Wrapper function for TargetRectDrawer::callBackFunc.
To make mouse call back function(TargetRectDrawer::callBackFunc)
inside the class, static wrapper function(TargetRectDrawer::wrapperCallBackFunc)
is needed.

----------------------------------------------------------------- */
void TargetRectDrawer::wrapperCallBackFunc(int event, int x, int y, int flags, void* userdata)
{
	TargetRectDrawer * drawer = (TargetRectDrawer *)userdata;

	// The real call back function
	drawer->CallBackFunc(event, x, y, flags);
}

/* -----------------------------------------------------------------

Function : drawInitRect

Draw initial target rectangle on the window.

----------------------------------------------------------------- */
std::vector<std::pair<cv::Rect, cv::Scalar>>& TargetRectDrawer::drawInitRect(cv::Mat& _mat_img)
{
	int keyboard = 0;  // keyboard input

	TargetRectDrawer * drawer = new TargetRectDrawer(); // This will be passed to callback function, WrapperCallBackFunc

														// Initialize
	drawer->initTargetRectDrawer(_mat_img);

	do
	{
		// Show image
		imshow("Tracking System", drawer->img_draw);

		// Register callback function and get mouse event
		cv::setMouseCallback("Tracking System", TargetRectDrawer::wrapperCallBackFunc, drawer);

		keyboard = cv::waitKey(0);

		// if press ESC key, delete last rectangle
		if (keyboard == ESC)
		{
			// Remove the last rectangle
			drawer->rect_vec.pop_back();

			// Change img_prev and img_draw with the original(with no rectangle) one
			drawer->img_prev = drawer->img_orig.clone();
			drawer->img_draw = drawer->img_orig.clone();

			// Draw rectangle again except the last rectangle
			cout << "Draw rect again" << endl;
			std::for_each(drawer->rect_vec.begin(), drawer->rect_vec.end(), [&](std::pair<cv::Rect, cv::Scalar> _pair) {
				cout << _pair.first << endl;
				cv::rectangle(drawer->img_prev, _pair.first, _pair.second, 2);
				cv::rectangle(drawer->img_draw, _pair.first, _pair.second, 2);
			});
		}
		// if press ENTER key, finish drawing.
	} while (keyboard != ENTER);

	// return drawing result
	return drawer->getRectVec();
}

/* ==========================================================================

Class : FrameReader

FrameReader is responsible for reading all images from designated directory

========================================================================== */
class FrameReader
{
private:
	std::string   path;    // Frame path
	std::string   image_type;  // Image type (For now, just "jpg" is available)
	_finddata_t   fd;
	intptr_t   handle;
	int     status;     // Reading status (CANNOT_OPEN_DIR = 3 / WAITING_FOR_READING_FRAME = 4 / READING_FRAME_STARTED = 5 / READING_FRAME_DONE = 6)

public:

	FrameReader() : status(FRAME_READER_IS_MADE) {};

	/* Set Function */
	void setPath(string _path) { this->path.assign(_path); }
	void setImageType(string _image_type) { this->image_type.assign(_image_type); }
	void setStatus(int status) { this->status = status; }


	/* Get Function*/
	std::string getPath() { return this->path; }
	std::string getImageType() { return this->image_type; }
	_finddata_t getFileFinder() { return this->fd; }
	intptr_t getHandler() { return this->handle; }
	int   getStatus() { return this->status; }

	/* Core Function */
	// Intialize FrameReader object
	int initFrameReader(std::string _path, std::string _image_type);

	// Get next frame image
	int getNextFrame(cv::Mat& mat_img);
};


/* -----------------------------------------------------------------------------------

Function : initFrameReader

Initialize FrameReader.

----------------------------------------------------------------------------------- */
int FrameReader::initFrameReader(std::string _path, std::string _image_type)
{
	// Get the first image in the _path
	// If success to complete this function, fd is pointing to the first frame in the FrameReader::path
	handle = _findfirst((_path + "/*." + _image_type).c_str(), &fd);

	// Cannot access to the _path
	if (handle == -1)
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : Constructor of FrameReader" << std::endl;
		std::cout << "Cannot open image file named _path with _image_type" << std::endl;
		std::cout << "=============================================================" << std::endl;

		// Change Status CANNOT_OPEN_DIR
		this->status = CANNOT_OPEN_DIR;

		return FAIL;
	}
	// Success to get the first image 
	else
	{
		// Set the FrameReader::path
		this->path.assign(_path);
		// Set the FrameReader::image_type
		this->image_type.assign(_image_type);
		// Change Status WAITING_FOR_READING_FRAME
		this->status = WAITING_FOR_READING_FRAME;

		return SUCCESS;
	}
}

/* -----------------------------------------------------------------------------------

Function : getNextFrame

Get the next frame image from the FrameReader::path

----------------------------------------------------------------------------------- */
int FrameReader::getNextFrame(cv::Mat& mat_img)
{
	switch (status)
	{
		// If fail to access to the path
	case CANNOT_OPEN_DIR:
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int FrameReader::getNextFrame" << std::endl;
		std::cout << "Wrong Path!" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;

		// Initialize is done. Now, FrameReader::fd is pointing to the 'First' frame and we need to get this 'First' frame
	case WAITING_FOR_READING_FRAME:
		mat_img = cv::imread(this->path + "/" + this->fd.name, CV_LOAD_IMAGE_COLOR);

		// Change status from WAITING_FOR_READING_FRAME to READING_FRAME_STARTED
		status = READING_FRAME_STARTED;

		return SUCCESS;

		// Now, FrameReader::fd is pointing to the 'Previous' frame, so need to move FrameReadr::fd next and get the frame image
	case READING_FRAME_STARTED:
		// Move FrameReader::fd to the next
		int result = _findnext(handle, &fd);

		// There is no more image
		if (result == -1)
		{
			std::cout << "========================== Notice! ==========================" << std::endl;
			std::cout << "Function : getNextFrame of FrameReader" << std::endl;
			std::cout << "Image reading is done" << std::endl;
			std::cout << "=============================================================" << std::endl;

			// Change status READING_FRAME_STARTED to READING_FRAME_DONE
			status = READING_FRAME_DONE;
			return FAIL;
		}
		else
		{
			mat_img = cv::imread(this->path + "/" + this->fd.name, CV_LOAD_IMAGE_COLOR);
			return SUCCESS;
		}
	}
}

/* ==========================================================================

Class : SingleTracker

This class is aim to track 'One' target for running time.
'One' SingleTracker object is assigned to 'One' person(or any other object).
In other words, if you are trying to track 'Three' people,
then need to have 'Three' SingleTracker object.

========================================================================== */
class SingleTracker
{
private:
	int    target_id;    // Unique Number for target
	double   confidence;    // Confidence of tracker
	cv::Rect  rect;     // Initial Rectangle for target
	cv::Point  center;     // Current center point of target
	bool   is_tracking_started; // Is tracking started or not? (Is initializing done or not?)
	cv::Scalar color;  // Box color

public:
	dlib::correlation_tracker tracker;  // Correlation tracker

										/* Member Initializer & Constructor*/
	SingleTracker(int _target_id, cv::Rect _init_rect, cv::Scalar _color)
		: target_id(_target_id), confidence(0), is_tracking_started(false)
	{
		// Exception
		if (_init_rect.area() == 0)
		{
			std::cout << "======================= Error Occured! ======================" << std::endl;
			std::cout << "Function : Constructor of SingleTracker" << std::endl;
			std::cout << "Parameter cv::Rect _init_rect's area is 0" << std::endl;
			std::cout << "=============================================================" << std::endl;
		}
		else
		{
			// Initialize rect and center using _init_rect
			this->setRect(_init_rect);
			this->setCenter(_init_rect);
			this->setColor(_color);
		}
	}

	/* Get Function */
	int    getTargetID() { return this->target_id; }
	cv::Rect     getRect() { return this->rect; }
	cv::Point    getCenter() { return this->center; }
	double   getConfidence() { return this->confidence; }
	bool   getIsTrackingStarted() { return this->is_tracking_started; }
	cv::Scalar getColor() { return this->color; }

	/* Set Function */
	void setTargetId(int _target_id) { this->target_id = _target_id; }
	void setRect(cv::Rect _rect) { this->rect = _rect; }
	void setRect(dlib::drectangle _drect) { this->rect = cv::Rect(_drect.tl_corner().x(), _drect.tl_corner().y(), _drect.width(), _drect.height()); }
	void setCenter(cv::Point _center) { this->center = _center; }
	void setCenter(cv::Rect _rect) { this->center = cv::Point(_rect.x + (_rect.width) / 2, _rect.y + (_rect.height) / 2); }
	void setCenter(dlib::drectangle _drect) { this->center = cv::Point(_drect.tl_corner().x() + (_drect.width() / 2), _drect.tl_corner().y() + (_drect.height() / 2)); }
	void setConfidence(double _confidence) { this->confidence = _confidence; }
	void setIsTrackingStarted(bool _b) { this->is_tracking_started = _b; }
	void setColor(cv::Scalar _color) { this->color = _color; }
	/* Core Function */
	// Initialize
	int startSingleTracking(cv::Mat& _mat_img);

	// Do tracking
	int doSingleTracking(cv::Mat& _mat_img);

	// Check the target is inside of the frame
	int isTargetInsideFrame(int _frame_width, int _frame_height);
};


/* ---------------------------------------------------------------------------------

Function : startSingleTracking

Initialize dlib::correlation_tracker tracker using dlib::start_track function

---------------------------------------------------------------------------------*/
int SingleTracker::startSingleTracking(cv::Mat& _mat_img)
{
	// Exception
	if (_mat_img.empty())
	{
		std::cout << "====================== Error Occured! =======================" << std::endl;
		std::cout << "Function : int SingleTracker::startSingleTracking" << std::endl;
		std::cout << "Parameter cv::Mat& _mat_img is empty image!" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// Convert _mat_img to dlib::array2d<unsigned char>
	dlib::array2d<unsigned char> dlib_frame = Util::cvtMatToArray2d(_mat_img);

	// Convert SingleTracker::rect to dlib::drectangle
	dlib::drectangle dlib_rect = Util::cvtRectToDrect(this->getRect());

	// Initialize SingleTracker::tracker
	this->tracker.start_track(dlib_frame, dlib_rect);
	this->setIsTrackingStarted(true);

	return SUCCESS;
}

/*---------------------------------------------------------------------------------

Function : isTargetInsideFrame

Check the target is inside the frame
If the target is going out of the frame, need to SingleTracker stop that target.

---------------------------------------------------------------------------------*/
int SingleTracker::isTargetInsideFrame(int _frame_width, int _frame_height)
{
	int cur_x = this->getCenter().x;
	int cur_y = this->getCenter().y;

	bool is_x_inside = ((0 <= cur_x) && (cur_x < _frame_width));
	bool is_y_inside = ((0 <= cur_y) && (cur_y < _frame_height));

	if (is_x_inside && is_y_inside)
		return TRUE;
	else
		return FALSE;
}

/* ---------------------------------------------------------------------------------

Function : doSingleTracking

Track 'one' target specified by SingleTracker::rect in a frame.
(It means that you need to call doSingleTracking once per a frame)
SingleTracker::rect is initialized to the target position in the constructor of SingleTracker
Using correlation_tracker in dlib, start tracking 'one' target

--------------------------------------------------------------------------------- */
int SingleTracker::doSingleTracking(cv::Mat& _mat_img)
{
	//Exception
	if (_mat_img.empty())
	{
		std::cout << "====================== Error Occured! ======================= " << std::endl;
		std::cout << "Function : int SingleTracker::doSingleTracking" << std::endl;
		std::cout << "Parameter cv::Mat& _mat_img is empty image!" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// Convert _mat_img to dlib::array2d<unsigned char>
	dlib::array2d<unsigned char> dlib_img = Util::cvtMatToArray2d(_mat_img);

	// Track using dlib::update function
	double confidence = this->tracker.update(dlib_img);

	// New position of the target
	dlib::drectangle updated_rect = this->tracker.get_position();

	// Update variables(center, rect, confidence)
	this->setCenter(updated_rect);
	this->setRect(updated_rect);
	this->setConfidence(confidence);

	return SUCCESS;
}

/* ==========================================================================

Class : TrackerManager

TrackerManager is aim to manage vector<std::shared_ptr<SingleTracker>>
for multi-object tracking.
(To make it easy, it's almost same with vector<SigleTracker *>)
So, this class provides insert, find, delete function.

========================================================================== */
class TrackerManager
{
private:
	std::vector<std::shared_ptr<SingleTracker>> tracker_vec; // Vector filled with SingleTracker shared pointer. It is the most important container in this program.

public:
	/* Get Function */
	std::vector<std::shared_ptr<SingleTracker>>& getTrackerVec() { return this->tracker_vec; } // Return reference! not value!

																							   /* Core Function */
																							   // Insert new SingleTracker shared pointer into the TrackerManager::tracker_vec
	int insertTracker(cv::Rect _init_rect, cv::Scalar _color, int _target_id);
	int insertTracker(std::shared_ptr<SingleTracker> new_single_tracker);

	// Find SingleTracker in the TrackerManager::tracker_vec using SingleTracker::target_id
	int findTracker(int _target_id);

	// Deleter SingleTracker which has ID : _target_id from TrackerManager::tracker_vec
	int deleteTracker(int _target_id);
};

/* -------------------------------------------------------------------------

Function : insertTracker

Create new SingleTracker object and insert it to the vector.
If you are about to track new person, need to use this function.

------------------------------------------------------------------------- */

int TrackerManager::insertTracker(cv::Rect _init_rect, cv::Scalar _color, int _target_id)
{
	// Exceptions
	if (_init_rect.area() == 0)
	{
		std::cout << "======================= Error Occured! ====================== " << std::endl;
		std::cout << "Function : int SingleTracker::initTracker" << std::endl;
		std::cout << "Parameter cv::Rect _init_rect's area is 0" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// if _target_id is already exists
	int result_idx = findTracker(_target_id);

	if (result_idx != FAIL)
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int SingleTracker::initTracker" << std::endl;
		std::cout << "_target_id already exists!" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// Create new SingleTracker object and insert it to the vector
	std::shared_ptr<SingleTracker> new_tracker(new SingleTracker(_target_id, _init_rect, _color));
	this->tracker_vec.push_back(new_tracker);

	return SUCCESS;
}

// Overload of insertTracker
int TrackerManager::insertTracker(std::shared_ptr<SingleTracker> new_single_tracker)
{

	//Exception
	if (new_single_tracker == nullptr)
	{
		std::cout << "======================== Error Occured! ===================== " << std::endl;
		std::cout << "Function : int TrackerManager::insertTracker" << std::endl;
		std::cout << "Parameter shared_ptr<SingleTracker> new_single_tracker is nullptr" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// if _target_id is already exists
	int result_idx = findTracker(new_single_tracker.get()->getTargetID());
	if (result_idx != FAIL)
	{
		std::cout << "====================== Error Occured! =======================" << std::endl;
		std::cout << "Function : int SingleTracker::insertTracker" << std::endl;
		std::cout << "_target_id already exists!" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// Insert new SingleTracker object into the vector
	this->tracker_vec.push_back(new_single_tracker);

	return SUCCESS;
}

/* -----------------------------------------------------------------------------------

Function : findTracker

Find SingleTracker object which has ID : _target_id in the TrackerManager::tracker_vec
If success to find return that iterator, or return TrackerManager::tracker_vec.end()

----------------------------------------------------------------------------------- */
int TrackerManager::findTracker(int _target_id)
{
	auto target = find_if(tracker_vec.begin(), tracker_vec.end(), [&, _target_id](auto ptr) -> bool {
		return (ptr.get()->getTargetID() == _target_id);
	});

	if (target == tracker_vec.end())
		return FAIL;
	else
		return target - tracker_vec.begin();
}

/* -----------------------------------------------------------------------------------

Function : deleteTracker

Delete SingleTracker object which has ID : _target_id in the TrackerManager::tracker_vec

----------------------------------------------------------------------------------- */
int TrackerManager::deleteTracker(int _target_id)
{
	int result_idx = this->findTracker(_target_id);

	if (result_idx == FAIL)
	{
		std::cout << "======================== Error Occured! =====================" << std::endl;
		std::cout << "Function : int TrackerManager::deleteTracker" << std::endl;
		std::cout << "Cannot find given _target_id" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}
	else
	{
		// Memory deallocation
		this->tracker_vec[result_idx].reset();

		// Remove SingleTracker object from the vector
		this->tracker_vec.erase(tracker_vec.begin() + result_idx);
		return SUCCESS;
	}
}

/* ===================================================================================================

Class : TrackingSystem

TrackingSystem is the highest-ranking manager in this program.
It uses FrameReader class to get the frame images, TrackerManager class for the
smooth tracking. And SingleTracker object will be included in TrackerManager::tracker_vec.
In each of SingleTracker, SingleTracker::startSingleTracking and SingleTracker::doSingleTracking
functios are taking care of tracking each target.
TrackingSystem is using these classes properly and hadling all expected exceptions.

====================================================================================================== */
class TrackingSystem
{
private:
	std::string  frame_path;  // Path to the frame image
	int    frame_width; // Frame image width
	int    frame_height; // Frame image height
	cv::Mat   current_frame; // Current frame
	std::vector<std::pair<cv::Rect, cv::Scalar>> init_target;

	TargetRectDrawer drawer; // TargetRectDrawer
	TrackerManager manager;  // TrackerManager
	FrameReader  reader;   // FrameReader

public:
	/* Constructor */
	TrackingSystem(std::string _frame_path)
	{
		// Set frame path
		this->frame_path.assign(_frame_path);

		// Initialize TrackingSystem::reader
		this->reader.initFrameReader(_frame_path, "jpg");

		cv::Mat temp_img = cv::imread(_frame_path + "/" + this->reader.getFileFinder().name, CV_LOAD_IMAGE_COLOR);
		this->setFrameWidth(temp_img.cols);
		this->setFrameHeight(temp_img.rows);

		this->init_target = this->drawer.drawInitRect(temp_img);
	};

	/* Get Function */
	std::string  getFramePath() { return this->frame_path; }
	int    getFrameWidth() { return this->frame_width; }
	int    getFrameHeight() { return this->frame_height; }
	cv::Mat   getCurrentFrame() { return this->current_frame; }
	TrackerManager getTrackerManager() { return this->manager; }
	FrameReader  getFrameReader() { return this->reader; }

	/* Set Function */
	void   setFramePath(std::string _frame_path) { this->frame_path.assign(_frame_path); }
	void   setFrameWidth(int _frame_width) { this->frame_width = _frame_width; }
	void   setFrameHeight(int _frame_height) { this->frame_height = _frame_height; }
	void   setCurrentFrame(cv::Mat _current_frame) { this->current_frame = _current_frame; }

	/* Core Function */
	// Initialize TrackingSystem.
	template<typename Container>
	int initTrackingSystem(Container _ids, Container _recs, Container _colors);
	int initTrackingSystem(int _target_id, cv::Rect _rect, cv::Scalar _color);
	int initTrackingSystem();
	// Start tracking
	int startTracking(cv::Mat& _mat_img);
	int startTracking(int _target_id, cv::Mat& _mat_img);

	// Draw tracking result
	int drawTrackingResult(cv::Mat& _mat_img);

	// Terminate program
	int terminateSystem();
};


/* -----------------------------------------------------------------------------------

Function : initTrackingSystem(overloaded)

Insert a SingleTracker object to the manager.tracker_vec
If you want multi-object tracking, call this function multiple times like

initTrackingSystem(0, rect1);
initTrackingSystem(1, rect2);
initTrackingSystem(2, rect3);

Then, the system is ready to tracking three targets.

----------------------------------------------------------------------------------- */
int TrackingSystem::initTrackingSystem(int _target_id, cv::Rect _rect, cv::Scalar _color)
{
	if (this->manager.insertTracker(_rect, _color, _target_id) == FAIL)
	{
		std::cout << "====================== Error Occured! =======================" << std::endl;
		std::cout << "Function : int TrackingSystem::initTrackingSystem" << std::endl;
		std::cout << "Cannot insert new SingleTracker object to the vector" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;
	}
	else
		return TRUE;
}

/* -----------------------------------------------------------------------------------

Function : initTrackingSystem(overloaded)

Insert multiple SingleTracker objects to the manager.tracker_vec in once.
If you want multi-object tracking, call this function just for once like

vector<cv::Rect> rects;
// Insert all rects into the vector

vector<int> ids;
// Insert all target_ids into the vector

initTrackingSystem(ids, rects)

Then, the system is ready to track multiple targets.

----------------------------------------------------------------------------------- */
template<typename Container>
int TrackingSystem::initTrackingSystem(Container _ids, Container _recs, Container _colors)
{
	// _ids and _recs should have same size and be in same order
	if (_ids.size() != _recs.size())
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int TrackingSystem::initTrackingSystem" << std::endl;
		std::cout << "Numbers of _ids and _recs should be same" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;
	}

	// Get next frame image
	if (this->reader.getNextFrame(mat_img) == FAIL)
	{
		std::cout << "======================== Error Occured! =====================" << std::endl;
		std::cout << "Function : int TrackingSystem::initTrackingSystem" << std::endl;
		std::cout << "Fail to get next frame image" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// Check the image is empty
	if (mat_img.empty())
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int TrackingSystem::initTrackingSystem" << std::endl;
		std::cout << "Input image is empty" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// Make Tracker Vector
	for (int i = 0; i < _ids.size(); i++)
	{
		if (this->manager.insertTracker(_recs[i], _colors[i], _target_id[i]) == FAIL)
		{
			std::cout << "=================== Error Occured! ==========================" << std::endl;
			std::cout << "Function : int TrackingSystem::initTrackingSystem" << std::endl;
			std::cout << "Fail to complete insertTracker function" << std::endl;
			std::cout << "=============================================================" << std::endl;

			return FAIL;
		}
	}

	return SUCCESS;
}

int TrackingSystem::initTrackingSystem()
{
	//std::vector<cv::Rect> rect_vec;
	//std::vector<cv::Scalar> color_vec;
	//std::vector<int> id_vec;

	for (int i = 0; i < this->init_target.size(); i++)
	{
		//rect_vec.push_back(init_target[i].first);
		//color_vec.push_back(init_target[i].second);
		//id_vec.push_back(i);

		int init_result = initTrackingSystem(i, this->init_target[i].first, this->init_target[i].second);

		if (init_result == FAIL)
			return FAIL;
	}

	return SUCCESS;

}

/* -----------------------------------------------------------------------------------

Function : startTracking(overloaded)

Track just one target.
If you want to track multiple targets,

startTracking(0, _mat_img)
startTracking(1, _mat_img)
startTracking(2, _mat_img)
...

Target ID should be given by initTrackingSystem function

----------------------------------------------------------------------------------- */
int TrackingSystem::startTracking(int _target_id, cv::Mat& _mat_img)
{
	// Check the image is empty
	if (_mat_img.empty())
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int TrackingSystem::startTracking" << std::endl;
		std::cout << "Input image is empty" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;
	}

	// Check the manager.tracker_vec size to make it sure target exists
	if (manager.getTrackerVec().size() == 0)
	{
		std::cout << "========================= Notice! ===========================" << std::endl;
		std::cout << "Function int TrackingSystem::startTracking" << std::endl;
		std::cout << "There is no more target to track" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;
	}

	// Find the target from the manager.tracker_vec
	int result_idx = manager.findTracker(_target_id);

	// If there is no target which has ID : _target_id
	if (result_idx == FAIL)
	{
		std::cout << "======================== Error Occured! =====================" << std::endl;
		std::cout << "Function : int TrackingSystem::startTracking" << std::endl;
		std::cout << "Cannot find Target ID : " << _target_id << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}

	// If there is no problem with frame image, set the image as current_frame
	setCurrentFrame(_mat_img);

	// Convert Mat to dlib::array2d to use start_track function
	dlib::array2d<unsigned char> dlib_cur_frame = Util::cvtMatToArray2d(this->getCurrentFrame());

	// startSingleTracking. This function must be called for just once when tracking is started for initializing
	if (!(manager.getTrackerVec()[result_idx].get()->getIsTrackingStarted()))
		manager.getTrackerVec()[result_idx].get()->startSingleTracking(this->getCurrentFrame());

	// doSingleTracking                 
	manager.getTrackerVec()[result_idx].get()->doSingleTracking(this->getCurrentFrame());

	// If target is going out of the frame, delete that tracker.
	if (manager.getTrackerVec()[result_idx].get()->isTargetInsideFrame(this->getFrameWidth(), this->getFrameHeight()) == FALSE)
	{
		manager.deleteTracker(_target_id);

		std::cout << "=========================== Notice! =========================" << std::endl;
		std::cout << "Function int TrackingSystem::startTracking" << std::endl;
		std::cout << "Target ID : " << _target_id << " is going out of the frame." << std::endl;
		std::cout << "Target ID : " << _target_id << " is erased!" << std::endl;
		std::cout << "=============================================================" << std::endl;
	}

	return SUCCESS;
}

/* -----------------------------------------------------------------------------------

Function : startTracking(overloaded)

Track all targets.
You don't need to give target id for tracking.
This function will track all targets.

----------------------------------------------------------------------------------- */
int TrackingSystem::startTracking(cv::Mat& _mat_img)
{
	// Check the image is empty
	if (_mat_img.empty())
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int TrackingSystem::startTracking" << std::endl;
		std::cout << "Input image is empty" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;
	}

	// Convert _mat_img to dlib::array2d<unsigned char>
	dlib::array2d<unsigned char> dlib_cur_frame = Util::cvtMatToArray2d(_mat_img);

	// For all SingleTracker, do SingleTracker::startSingleTracking.
	// Function startSingleTracking shold be done before doSingleTracking
	std::for_each(manager.getTrackerVec().begin(), manager.getTrackerVec().end(), [&](std::shared_ptr<SingleTracker> ptr) {
		if (!(ptr.get()->getIsTrackingStarted()))
		{
			ptr.get()->startSingleTracking(_mat_img);
			ptr.get()->setIsTrackingStarted(true);
		}
	});

	// For all SingleTracker, do SingleTracker::doSingleTracker
	std::for_each(manager.getTrackerVec().begin(), manager.getTrackerVec().end(), [&](std::shared_ptr<SingleTracker> ptr) {
		ptr.get()->doSingleTracking(_mat_img);
	});

	// If target is going out of the frame, delete that tracker.
	std::for_each(manager.getTrackerVec().begin(), manager.getTrackerVec().end(), [&](std::shared_ptr<SingleTracker> ptr) {
		if (ptr.get()->isTargetInsideFrame(this->getFrameWidth(), this->getFrameHeight()) == FALSE)
		{
			int target_id = ptr.get()->getTargetID();
			manager.deleteTracker(target_id);

			std::cout << "========================== Notice! ==========================" << std::endl;
			std::cout << "Function int TrackingSystem::startTracking" << std::endl;
			std::cout << "Target ID : " << target_id << " is going out of the frame." << std::endl;
			std::cout << "Target ID : " << target_id << " is erased!" << std::endl;
			std::cout << "=============================================================" << std::endl;
		}
	});

	return SUCCESS;
}

/* -----------------------------------------------------------------------------------

Function : drawTrackingResult

Deallocate all memory and close the program.

----------------------------------------------------------------------------------- */
int TrackingSystem::drawTrackingResult(cv::Mat& _mat_img)
{
	TrackerManager manager = this->getTrackerManager();

	// Exception
	if (manager.getTrackerVec().size() == 0)
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : int TrackingSystem::drawTrackingResult" << std::endl;
		std::cout << "Nothing to draw" << std::endl;
		std::cout << "=============================================================" << std::endl;
		return FAIL;
	}

	std::for_each(manager.getTrackerVec().begin(), manager.getTrackerVec().end(), [&_mat_img](std::shared_ptr<SingleTracker> ptr) {

		// Draw all rectangles
		cv::rectangle(_mat_img, ptr.get()->getRect(), ptr.get()->getColor(), 2);

		cv::String text(std::string("Target ID : ") + std::to_string(ptr.get()->getTargetID()));
		cv::Point text_pos = ptr.get()->getRect().tl();
		text_pos.x = text_pos.x - 10;
		text_pos.y = text_pos.y - 5;

		// Put all target ids
		cv::putText(_mat_img,
			text,
			text_pos,
			CV_FONT_HERSHEY_PLAIN,
			1,
			ptr.get()->getColor(),
			2);
	});

	return SUCCESS;
}

/* -----------------------------------------------------------------------------------

Function : terminateSystem

Draw rectangle around the each target and put target id on rectangle.

----------------------------------------------------------------------------------- */
int TrackingSystem::terminateSystem()
{
	std::vector<std::shared_ptr<SingleTracker>> remaining_tracker = manager.getTrackerVec();

	// Memory deallocation
	std::for_each(remaining_tracker.begin(), remaining_tracker.end(),
		[](std::shared_ptr<SingleTracker> ptr) { ptr.reset(); });

	//for (auto iter = remaining_tracker.begin(); iter != remaining_tracker.end(); ++iter)
	// remaining_tracker.erase(iter);

	std::cout << "Close Tracking System..." << std::endl;

	return 0;
}

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
	if (argc == 1)
	{
		std::cout << "======================= Error Occured! ======================" << std::endl;
		std::cout << "Function : main" << std::endl;
		std::cout << "Command : solution YOUR_FRAME_IMAGE_PATH (ex. solution D:/frame_image_directory)" << std::endl;
		std::cout << "=============================================================" << std::endl;

		return FAIL;
	}
	cv::Mat mat_img;

	// Create TrackingSystem Object
	TrackingSystem system(argv[1]); 

	system.initTrackingSystem();

	FrameReader frame_reader = system.getFrameReader();

	// Track two targets
	while (true)
	{
		// Get next frame
		if (frame_reader.getNextFrame(mat_img) == FAIL)
			break;

		// Tracking
		if (system.startTracking(mat_img) == FAIL)
			break;

		// Draw tracking result
		if (system.getTrackerManager().getTrackerVec().size() != 0)
			system.drawTrackingResult(mat_img);

		cv::imshow("Tracking System", mat_img);
		cv::waitKey(1);
	}

	system.terminateSystem();
}
