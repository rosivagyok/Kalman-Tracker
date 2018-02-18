#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\video\video.hpp>
#include <iostream>
#include <Windows.h>
#include <cstdlib> 
#include <ctime>
#include <experimental/filesystem>
#include <filesystem>
#include "XmlWriter.h"
using namespace std::experimental::filesystem::v1;
using namespace std;
using namespace cv;

enum direction{ East = 0, SouthEast = 1, South = 2, SouthWest = 3, West = 4, NorthWest = 5, North = 6, NorthEast = 7 };
Mat clipFrame(Mat frame);
class MBLOB{
private:
	vector<Point> points;
	Point CoM;
	Point BBoxL, BBoxH;
public:
	MBLOB(){
		this->CoM = Point(0, 0);
		this->BBoxL = Point(-1, -1);
		this->BBoxH = Point(-1, -1);
	}
	void add(Point pt){
		if (!this->contains(pt)){
			if (points.size() == 0){
				this->BBoxL = pt;
				this->BBoxH = pt;
			}
			else{
				if (this->BBoxL.x > pt.x) this->BBoxL.x = pt.x;
				if (this->BBoxL.y > pt.y) this->BBoxL.y = pt.y;
				if (this->BBoxH.x < pt.x) this->BBoxH.x = pt.x;
				if (this->BBoxH.y < pt.y) this->BBoxH.y = pt.y;
			}
			this->points.push_back(pt);
			this->CoM.x += pt.x;
			this->CoM.y += pt.y;
		}
	}
	Point getCoM(){
		return this->CoM * ((double)1 / this->points.size());
	}
	vector<Point> getPoints() {
		return this->points;
	}
	void setPoint(int id, Point p) {
		this->points[id] = p;
	}
	bool contains(Point pt){
		return (find(this->points.begin(), this->points.end(), pt) == this->points.end()) ? false : true;
	}
	Point getBBoxL(){
		return this->BBoxL;
	}
	Point getBBoxH(){
		return this->BBoxH;
	}
	Rect getBBox(){
		return Rect(this->BBoxL, this->BBoxH);
	}
	void setBBoxH(Point boxH) & {
		this->BBoxH = boxH;
	}
	void setBBoxL(Point boxL) & {
		this->BBoxL = boxL;
	}
	void setCoM(Point com) & {
		this->CoM = com * (double) this->points.size();
	}
	MBLOB noBlob() {
		MBLOB m;
		m.add(Point(-1, -1));
		return m;
	}
	void resize(int i) {
		this->points.resize(i);
	}
	MBLOB equals(MBLOB m) {
		MBLOB n;
		n.setBBoxL(m.getBBoxL());
		n.setBBoxH(m.getBBoxH());
		n.setCoM(m.getCoM());
		for (int i = 0; i < m.getPoints().size(); i++) {
			n.setPoint(i, m.getPoints()[i]);
		}
		return n;
	}
};
struct grassResult{
	Mat img;
	MBLOB MBLOB;
};
class grassFireHandler{
private:
	vector<Point> toCheck; //list of pixels to check neighbours
	int offsetX;
	int offsetY;
	Mat* img;
	Point N, E, S, W;
public:
	grassFireHandler(){
		this->reset(true);
		//declare 4 main direction vectors
		N = Point(0, -1);
		E = Point(1, 0);
		S = Point(0, 1);
		W = Point(-1, 0);
	}
	void setSeed(Point seed){
		this->offsetX = seed.x;
		this->offsetY = seed.y;
	}
	void setImage(Mat* img){
		this->img = img;
	}
	void reset(bool img = false){
		this->offsetX = 0;
		this->offsetY = 0;
		if (img)this->img = NULL;
		this->toCheck.clear();
	}
	bool addPxToBl(MBLOB &MBLOB, Point bp){
		if (!MBLOB.contains(bp)){
			MBLOB.add(bp);
			this->toCheck.push_back(bp);
			return true;
		}
		return false;
	}
	grassResult runFromSeed(bool burn = false){
		Mat img = *this->img;
		MBLOB MBLOB;
		grassResult ret;
		if (img.at<uchar>(this->offsetY, this->offsetX) == 255){
			Point bp = Point(this->offsetX, this->offsetY);
			this->addPxToBl(MBLOB, bp);
		}
		while (this->toCheck.size() > 0){
			Point cp = this->toCheck[0];
			//check 8 directions
			//get the 8 directions from current Point
			Point n, nw, w, sw, s, se, e, ne;
			n = cp + N;
			nw = cp + N + W;
			w = cp + W;
			sw = cp + S + W;
			s = cp + S;
			se = cp + S + E;
			e = cp + E;
			ne = cp + N + E;
			if (n.x >= 0 && n.x < img.cols && n.y >= 0 && n.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(n) == 255) this->addPxToBl(MBLOB, n);
				if (burn) img.at<uchar>(n) = 0;
			}
			if (ne.x >= 0 && ne.x < img.cols && ne.y >= 0 && ne.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(ne) == 255) this->addPxToBl(MBLOB, ne);
				if (burn) img.at<uchar>(ne) = 0;
			}
			if (nw.x >= 0 && nw.x < img.cols && nw.y >= 0 && nw.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(nw) == 255) this->addPxToBl(MBLOB, nw);
				if (burn) img.at<uchar>(nw) = 0;
			}
			if (e.x >= 0 && e.x < img.cols && e.y >= 0 && e.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(e) == 255) this->addPxToBl(MBLOB, e);
				if (burn) img.at<uchar>(e) = 0;
			}
			if (w.x >= 0 && w.x < img.cols && w.y >= 0 && w.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(w) == 255) this->addPxToBl(MBLOB, w);
				if (burn) img.at<uchar>(w) = 0;
			}
			if (sw.x >= 0 && sw.x < img.cols && sw.y >= 0 && sw.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(sw) == 255) this->addPxToBl(MBLOB, sw);
				if (burn) img.at<uchar>(sw) = 0;
			}
			if (se.x >= 0 && se.x < img.cols && se.y >= 0 && se.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(se) == 255) this->addPxToBl(MBLOB, se);
				if (burn) img.at<uchar>(se) = 0;
			}
			if (s.x >= 0 && s.x < img.cols && s.y >= 0 && s.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(s) == 255) this->addPxToBl(MBLOB, s);
				if (burn) img.at<uchar>(s) = 0;
			}
			this->toCheck.erase(this->toCheck.begin());
		}
		ret.MBLOB = MBLOB;
		ret.img = img;
		return ret;
	}
};
class ROI{
protected:
	Point mp, error; //Error is point and not vector as we can use abs
	Mat image;
public:
	ROI(){}
	ROI(Point mp, Point errorVec){
		this->mp = mp;
		this->error = errorVec; //Assumes symmetric error
	}
	void setCenter(Point p){
		this->mp = p;
	}
	void setError(Point p){
		this->error = p;
	}
	Point getCenter(){
		return this->mp;
	}
	void extractFromImage(Mat img){
		Point lowEnd = Point(this->mp.x - this->error.x, this->mp.y - this->error.y);
		Point highEnd = Point(this->mp.x + 1 + this->error.x, this->mp.y + 1 + this->error.y);
		if (lowEnd.x < 0) lowEnd.x = 0;
		if (lowEnd.y < 0) lowEnd.y = 0;
		if (highEnd.x > img.cols - 1) highEnd.x = img.cols - 1;
		if (highEnd.y > img.rows - 1) highEnd.y = img.rows - 1;
		this->image = img(Range(lowEnd.y, highEnd.y), Range(lowEnd.x, highEnd.x));
	}
	Mat getImage(){
		return (this->image.empty()) ? Mat::zeros(this->image.size(),CV_8UC1): this->image;
	}
	bool isPtInROI(Point pt){
		Point lowEnd = Point(this->mp.x - this->error.x, this->mp.y - this->error.y);
		Point highEnd = Point(this->mp.x + 1 + this->error.x, this->mp.y + 1 + this->error.y);
		if (lowEnd.x < 0) lowEnd.x = 0;
		if (lowEnd.y < 0) lowEnd.y = 0;
		return (lowEnd.x <= pt.x && lowEnd.y <= pt.y && highEnd.y >= pt.y && highEnd.x >= pt.x) ? true : false;
	}
};
class BinaryImage{
protected:
	Mat bin_img;
	void dialate(int size){
		Mat temp = Mat::zeros(this->bin_img.size(), CV_8UC1);
		int rad = (size - 1) / 2;
		for (int y = rad; y < this->bin_img.rows - rad; y++){
			for (int x = rad; x < this->bin_img.cols - rad; x++){
				bool dialate = false;
				for (int j = -rad; j <= rad; j++){
					if (dialate) break;
					for (int i = -rad; i <= rad; i++){
						if (this->check(x + i, y + j)){
							dialate = true;
							break;
						}
					}
				}
				if (dialate){
					temp.at<uchar>(y, x) = 255;
				}
			}
		}
		this->bin_img = temp;
	}
	void dialate(int sizex, int sizey){
		Mat temp = Mat::zeros(this->bin_img.size(), CV_8UC1);
		int radx = (sizex - 1) / 2;
		int rady = (sizey - 1) / 2;
		for (int y = rady; y < this->bin_img.rows - rady; y++){
			for (int x = radx; x < this->bin_img.cols - radx; x++){
				bool dialate = false;
				for (int j = -rady; j <= rady; j++){
					if (dialate) break;
					for (int i = -radx; i <= radx; i++){
						if (this->check(x + i, y + j)){
							dialate = true;
							break;
						}
					}
				}
				if (dialate){
					temp.at<uchar>(y, x) = 255;
				}
			}
		}
		this->bin_img = temp;
	}
	void erode(int size){
		Mat temp = Mat::zeros(this->bin_img.size(), CV_8UC1);
		int rad = (size - 1) / 2;
		for (int y = rad; y < this->bin_img.rows - rad; y++){
			for (int x = rad; x < this->bin_img.cols - rad; x++){
				bool erode = true;
				for (int j = -rad; j <= rad; j++){
					if (!erode) break;
					for (int i = -rad; i <= rad; i++){
						if (!this->check(x + i, y + j)){
							erode = false;
							break;
						}
					}
									}
				if (erode){
					temp.at<uchar>(y, x) = 255;
				}
			}
		}
		this->bin_img = temp;
	}
	void erode(int sizex, int sizey){
		Mat temp = Mat::zeros(this->bin_img.size(), CV_8UC1);
		int radx = (sizex - 1) / 2;
		int rady = (sizey - 1) / 2;
		for (int y = rady; y < this->bin_img.rows - rady; y++){
			for (int x = radx; x < this->bin_img.cols - radx; x++){
				bool erode = true;
				for (int j = -rady; j <= rady; j++){
					if (!erode) break;
					for (int i = -radx; i <= radx; i++){
						if (!this->check(x + i, y + j)){
							erode = false;
							break;
						}
					}

				}
				if (erode){
					temp.at<uchar>(y, x) = 255;
				}
			}
		}
		this->bin_img = temp;
	}
public:
	BinaryImage(Mat bin){
		this->bin_img = bin;
	}
	bool check(int x, int y){
		if (this->bin_img.cols > x && x >= 0 && this->bin_img.cols > y && y >= 0) return ((int)this->bin_img.at<uchar>(y, x) == 255) ? true : false;
		return false;
	}
	bool check(direction dir, int x, int y){
		switch (dir){
		case East:
			if (this->bin_img.cols > x + 1 && x + 1 >= 0 && y >= 0 && y < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y, x + 1) == 255) ? true : false;
			break;
		case SouthEast:
			if (this->bin_img.cols > x + 1 && x + 1 >= 0 && y + 1 >= 0 && y + 1 < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y + 1, x + 1) == 255) ? true : false;
			break;
		case South:
			if (this->bin_img.cols > x && x >= 0 && y + 1 >= 0 && y + 1 < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y + 1, x) == 255) ? true : false;
			break;
		case SouthWest:
			if (this->bin_img.cols > x - 1 && x - 1 >= 0 && y + 1 >= 0 && y + 1 < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y + 1, x - 1) == 255) ? true : false;
			break;
		case West:
			if (this->bin_img.cols > x - 1 && x - 1 >= 0 && y >= 0 && y < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y, x - 1) == 255) ? true : false;
			break;
		case NorthWest:
			if (this->bin_img.cols > x - 1 && x - 1 >= 0 && y - 1 >= 0 && y - 1 < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y - 1, x - 1) == 255) ? true : false;
			break;
		case North:
			if (this->bin_img.cols > x && x >= 0 && y - 1 >= 0 && y - 1 < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y - 1, x) == 255) ? true : false;
			break;
		case NorthEast:
			if (this->bin_img.cols > x + 1 && x + 1 >= 0 && y - 1 >= 0 && y - 1 < this->bin_img.rows) return ((int)this->bin_img.at<uchar>(y - 1, x + 1) == 255) ? true : false;
			break;

		default:
			return false;
			break;
		}
		return false;
	}
	void open(int size){
		this->erode(size);
		this->dialate(size);
	}
	void open(int x, int y){
		this->erode(x, y);
		this->dialate(x, y);
	}
	void close(int size){
		this->dialate(size);
		this->erode(size);
	}
	void close(int x, int y){
		this->dialate(x, y);
		this->erode(x, y);
	}
	Mat get(){
		return this->bin_img;
	}
};

class Image{ //Only GS
protected:
	Mat im;
	Mat bg;
	bool isVideo;
	VideoCapture cap;
	int stage;
	string file;
	int id;
public:
	Image(string file) {
		this->isVideo = true;
		this->cap = VideoCapture(file);
		this->stage = 0;
		this->file = file;
		this->id = -1;
		while (!this->cap.isOpened()) {}
	}
	Image(string file, bool video0_image1){
		if (video0_image1 == 0) {
			this->isVideo = true;
			this->cap = VideoCapture(file);
			this->stage = 0;
			this->file = file;
			this->id = -1;
			while (!this->cap.isOpened()) {}
		}
		else {
			this->isVideo = false;
			this->cap = VideoCapture(file);
			this->cap.set(CV_CAP_PROP_BRIGHTNESS, 215);
			this->cap.set(CV_CAP_PROP_CONTRAST, 165);
			this->cap.set(CV_CAP_PROP_SATURATION, 0);
			this->stage = 0; //Get bg
			this->file = file;
			this->id = -1;
			while (!this->cap.isOpened()) {}
		}
	}
	Image(int id){
		this->isVideo = false;
		this->cap = VideoCapture(id);
		this->cap.set(CV_CAP_PROP_BRIGHTNESS, 215);
		this->cap.set(CV_CAP_PROP_CONTRAST, 165);
		this->cap.set(CV_CAP_PROP_SATURATION, 0);
		this->stage = 0; //Get bg
		this->file = "";
		this->id = id;
		while (!this->cap.isOpened()){}
	}
	Size getSize(){
		return this->bg.size();
	}

	void setBG(string path) {
		bg = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
	}

	void getBG(int skip = -1){
		bool selected = false;
		//Mat frame = imread("bg.png", CV_LOAD_IMAGE_GRAYSCALE);
		Mat frame;
		//cout << frame.size() << endl; system("PAUSE");
		while (!selected){
			for (int i = 0; i < skip; i++) {
				this->cap.grab();
			}
			this->cap.read(frame);
			while (frame.empty()){
				this->cap.read(frame);
			}
			//cout << frame.size() << endl; system("PAUSE");
			//frame = clipFrame(frame);
			//cvtColor(frame, this->bg, CV_BGR2GRAY);
			this->bg = frame;
			imshow("Presss space to select bg, anything else to skip frame", frame);
			if (waitKey(0) == 32){
				selected = true;
			}
		}
		this->cap.release();
		(id == -1) ? this->cap.open(file) : (this->cap.open(id));
		destroyAllWindows();
		return;
	}
	Mat returnBG(){
		return this->bg;
	}
	Mat getNextFrame(int skip = -1){
		Mat frame;
		while (frame.empty()){
			for (int i = 0; i < skip; i++){
				this->cap.grab();
			}
			this->cap.read(frame);
		}
		//cvtColor(frame, frame, CV_BGR2GRAY);
		return frame;
	}
	Mat subtractBG(Mat frame)
	{
		Mat bg = this->returnBG();
		Mat out = Mat(bg.size(), CV_8UC1);

		for (int j = 0; j < out.rows; j++)
		{
			for (int i = 0; i < out.cols; i++)
			{
				out.at<uchar>(j, i) = abs((int)frame.at<uchar>(j, i) - (int)bg.at<uchar>(j, i));
			}
		}
		return out;
	}
	BinaryImage binarize(Mat im, int thresh){
		Mat temp;
		threshold(im, temp, thresh, 255, CV_THRESH_BINARY);
		//adaptiveThreshold(im, temp, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 5, -3);
		BinaryImage bin(temp);
		return bin;
	}
	Mat reduceNoise(Mat im, int rad){
		Mat out = Mat::zeros(im.size(), CV_8UC1);
		for (int y = rad; y < im.rows - rad; y++){
			for (int x = rad; x < im.cols - rad; x++){
				vector<int> pixelList;
				for (int i = -rad; i <= rad; i++){
					for (int j = -rad; j <= rad; j++){
						pixelList.push_back(im.at<uchar>(y + j, x + i));
					}
				}
				sort(pixelList.begin(), pixelList.end());
				int total = 2 * rad + 1;
				out.at<uchar>(y, x) = pixelList[(total - 1 / 2)];
			}
		}
		return out;
	}
};
class MBLOBMemory{
private:
	KalmanFilter kf;
	ROI MBLOBROI;
	int error;
	int keep;
public:
	MBLOBMemory(Point pt){
		this->kf = KalmanFilter(4, 2, 0);
		this->kf.transitionMatrix = *(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		this->kf.statePre.at<float>(0) = pt.x;
		this->kf.statePre.at<float>(1) = pt.y;
		this->kf.statePre.at<float>(2) = 0;
		this->kf.statePre.at<float>(3) = 0;
		setIdentity(this->kf.measurementMatrix);
		setIdentity(this->kf.processNoiseCov, Scalar::all(1e-3));
		//setIdentity(this->kf.processNoiseCov, Scalar::all(0));
		setIdentity(this->kf.measurementNoiseCov, Scalar::all(3));
		setIdentity(this->kf.errorCovPost, Scalar::all(.1));
		Mat_<float> measurement(2, 1);
		measurement.at<float>(0) = pt.x;
		measurement.at<float>(1) = pt.y;
		this->kf.correct(measurement);
		this->MBLOBROI.setError(Point(50, 50));
		this->MBLOBROI.setCenter(pt);
		this->keep = 0;
	}
	Point predict(){
		Mat pred = this->kf.predict();
		Point predict = Point(pred.at<float>(0), pred.at<float>(1));
		//cout << "predict" << predict << endl;
		this->updateROICenter(predict);
		return predict;
	}
	ROI getRoi(){
		return this->MBLOBROI;
	}
	void updateROICenter(Point center){
		this->MBLOBROI.setCenter(center);
	}

	void updateROIError(Point error){
		this->MBLOBROI.setError(error);
	}
	Point estimated(Point measure, int width = 40){
		//cout << "Measurement: " << measure;
		Mat_<float> measurement(2, 1);
		Mat pre = this->kf.statePre;
		measurement.at<float>(0) = measure.x;
		measurement.at<float>(1) = measure.y;
		Mat estimate = this->kf.correct(measurement);
		Mat error = pre - estimate+40;
		this->updateROIError(Point(error.at<float>(0), error.at<float>(1)));
		return Point(estimate.at<float>(0), estimate.at<float>(1));
	}
	void disappear(){
		this->keep++;
	}
	void reappear(){
		this->keep = 0;
	}
	bool toDelete(){
		return keep > 8 ? true : false;
	}
};
class memoryTracker{
private:
	vector<MBLOBMemory> mem;
	vector<Point> prevPoint;
	vector<int> inPic;
	vector<Scalar> color;
	static Scalar randomColor(RNG& rng)
	{
		int icolor = (unsigned)rng;
		return Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
	}
public:
	memoryTracker(){

	}
	int getMemSize(){
		return this->mem.size();
	}
	MBLOBMemory getMem(int id){
		return this->mem[id];
	}

	void memReappear(int id){
		this->mem[id].reappear();
		this->inPic[id]++;
	}

	void memDisappear(int id){
		this->mem[id].disappear();
	}
	ROI getMemROI(int id){
		this->mem[id].predict();
		return this->mem[id].getRoi();
	}
	Point addMeasurementTo(int id, Point measure, int width = 40, bool IsHidden = false){
		Point est = this->mem[id].estimated(measure,width);
		if (!IsHidden)this->prevPoint[id] = est;
		return est;
	}
	Point getPrevPoint(int id){
		return this->prevPoint[id];
	}
	void addNewMem(Point pt){
		this->mem.push_back(MBLOBMemory(pt));
		this->prevPoint.push_back(pt);
		this->inPic.push_back(1);
		RNG rng = RNG(time(0)+3*time(0));
		this->color.push_back(this->randomColor(rng));
	}
	void cleanUp(){
		for (int i = 0; i < this->mem.size(); i++){
			if (this->mem[i].toDelete()){
				/*cout << " delete " << i;
				system("PAUSE");*/
				this->mem.erase(this->mem.begin() + i);
				this->prevPoint.erase(this->prevPoint.begin() + i);
				this->color.erase(this->color.begin() + i);
				this->inPic.erase(this->inPic.begin() + i);
				i--;
			}
		}
	}
	Scalar getColor(int id){
		return this->color[id];
	}
	void predict(){
		for (int i = 0; i < this->mem.size(); i++) mem[i].predict();
	}
	bool isEmpty(){
		return this->mem.size() == 0 ? true : false;
	}
	bool toDraw(int id){
		return (this->inPic[id] > 6)?true:false;
	}
};

bool sortSecondary( vector<float> i, vector<float> j){
	return (i[1] < j[1]);
}

struct Priority{
	int id;
	float dist;
	bool isInRoI;
};
class Priorities{
private:
	vector<Priority> priorities;
public:
	static bool sortIt(Priority i, Priority j){
		if (i.isInRoI == j.isInRoI)
		return (i.dist < j.dist);
		else{
			if (i.isInRoI) return true; else return false;
		}
	}
	Priorities(){}
	void addPriority(Priority p){
		this->priorities.push_back(p);
	}
	Priority getPriority(int id){
		return this->priorities[id];
	}
	void sortPriorities(){
		sort(this->priorities.begin(), this->priorities.end(), sortIt);
	}
	int size(){
		return this->priorities.size();
	}
	void removePriority(int id){
		this->priorities.erase(this->priorities.begin() + id);
	}
};
Mat clipFrame(Mat frame){
	return frame.rowRange(20, frame.rows);
}

void convert_to_RGB(const path& directory, string filterFileType)
{
	if (exists(directory))
	{
		directory_iterator end;
		for (directory_iterator iter(directory); iter != end; ++iter)
			if ((is_regular_file(*iter) && iter->path().extension().string() == filterFileType))
			{
				String path = (iter->path()).generic_string();
				Mat frame = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
				imwrite(path, frame);
			}
	}
}


void img_defects(Mat binImg) {

	Mat draw = Mat::zeros(binImg.size(), CV_8UC3);
	vector<Mat> bin(3);
	bin.at(0) = binImg; //for blue channel
	bin.at(1) = binImg;   //for green channel
	bin.at(2) = binImg;
	merge(bin,draw);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(binImg, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	vector<vector<Point>> hull(contours.size());
	vector<vector<int>> hullsI(contours.size());
	vector<vector<Vec4i>> defects(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hull[i], false);
		convexHull(Mat(contours[i]), hullsI[i], false);
		if (hullsI[i].size() > 3) convexityDefects(contours[i], hullsI[i], defects[i]);

	}

	for (int i = 0; i< contours.size(); i++)
	{
		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
		drawContours(draw, hull, i, Scalar(0, 0, 255), 1, 8, vector<Vec4i>(), 0, Point());
	}

	for (int i = 0; i < contours.size(); i++)
	{
		Vec4i tmp = Vec4i(0, 0, 0, 0);
		vector<Vec4i>::iterator d = defects[i].begin();
		while (d != defects[i].end()) {
			Vec4i& v = (*d);
			d++;
			Point ptFar(contours[i][v[2]]);
			circle(draw, ptFar, 4, Scalar(0, 255, 0), 2);
		}
	}
	//imshow("DEF", draw);
}

vector<MBLOB> defects(MBLOB blob, Mat grad_x, Mat grad_y, Mat test, int type) {


	vector<MBLOB> mblobs;
	vector<vector<Point>> contours;
	Mat blobMat;
	blobMat = Mat::zeros(blob.getBBox().height, blob.getBBox().width, CV_8U);
	for (int i = 0; i < blobMat.size().height; i++)
	{
		for (int j = 0; j < blobMat.size().width; j++) {
			Point p = Point(blob.getBBoxL().x + j, blob.getBBoxL().y + i);
			if (blob.contains(p)) blobMat.at<uchar>(i, j) = 255;
			else blobMat.at<uchar>(i, j) = 0;
		}
	}
	Mat blob_mat;
	blobMat.copyTo(blob_mat);
	vector<Vec4i> hierarchy;
	findContours(Mat(blobMat), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
	vector<vector<Point>> hull(contours.size());
	vector<vector<int>> hullsI(contours.size());
	vector<vector<Vec4i>> defects(contours.size());
	for (int i = 0; i < contours.size(); i++)
	{
		convexHull(Mat(contours[i]), hullsI[i], false);
		convexHull(Mat(contours[i]), hull[i], false);
		if (hullsI[i].size() > 3) convexityDefects(contours[i], hullsI[i], defects[i]);
	}

	for (int i = 0; i < contours.size(); i++)
	{
		Vec4i tmp = Vec4i(0, 0, 0, 0);
		Point ptFar(-1,-1);
		vector<Vec4i>::iterator d = defects[i].begin();
		int tmp0 = blob.getBBox().height;
		Point tmp1(-1, -1);
		if (type == 0) {
			while (d != defects[i].end()) {
				Vec4i& v = (*d);
				if (contours[i][v[0]].y < tmp0 || contours[i][v[1]].y < tmp0) {
					tmp1 = contours[i][v[0]];
					tmp0 = min(contours[i][v[0]].y, contours[i][v[1]].y);
				}
				d++;
			}
		}
		d = defects[i].begin();
		while (d != defects[i].end()) {
			Vec4i& v = (*d);
			float depth = float(v[3]) / 256.f;
			if (depth > (float(tmp[3])/256.f) && depth > 5) {
				tmp = v;
				if (type == 1) {
					//cout << blob.getBBoxL().x + contours[i][tmp[2]].x << endl;
					if (grad_x.at<uchar>(blob.getBBoxL().y + contours[i][tmp[2]].y, blob.getBBoxL().x + contours[i][tmp[2]].x) > 0){// || grad_x.at<uchar>(blob.getBBoxL().x - 1 + contours[i][tmp[2]].x - 1, blob.getBBoxL().y - 1 + contours[i][tmp[2]].y) > 0 || grad_x.at<uchar>(blob.getBBoxL().x - 1 + contours[i][tmp[2]].x + 1, blob.getBBoxL().y - 1 + contours[i][tmp[2]].y) > 0) {
						ptFar = contours[i][tmp[2]];
					}
				}
				else {
					if (contours[i][v[0]] == tmp1) {
						if (grad_y.at<uchar>(blob.getBBoxL().y + contours[i][tmp[2]].y, blob.getBBoxL().x + contours[i][tmp[2]].x) > 0) {// || grad_y.at<uchar>(blob.getBBoxL().x - 1 + contours[i][tmp[2]].x, blob.getBBoxL().y - 1 + contours[i][tmp[2]].y - 1) > 0 || grad_y.at<uchar>(blob.getBBoxL().x - 1 + contours[i][tmp[2]].x, blob.getBBoxL().y - 1 + contours[i][tmp[2]].y + 1) > 0) {
							ptFar = contours[i][tmp[2]];
						}
					}
				}
			}
			d++;
		}

		vector<MBLOB> new_blobs;

		
		if (ptFar.x != -1) {
			circle(blobMat, ptFar, 4, Scalar(0, 255, 0), 2);
			if (ptFar.x != 0 && ptFar.x != blob.getBBox().width - 1 && ptFar.y != 0 && ptFar.y != blob.getBBox().height - 1) {
				if (type == 0) {
					int pos_ym = ptFar.y - 1;
					int pos_yp = ptFar.y + 1;
					bool stopm = false;
					bool stopp = false;
					blob_mat.at<uchar>(ptFar.y, ptFar.x) = 0;
					for (int i = 0; i < blob.getBBox().height; i++) {
						if (stopp && stopm) break;
						if (pos_ym < 0) stopm = true;
						if (pos_yp > blob.getBBox().height-1) stopp = true;
						if (!stopm && blob_mat.at<uchar>(pos_ym, ptFar.x) == 255) {
							blob_mat.at<uchar>(pos_ym, ptFar.x) = 0;
							pos_ym--;
						}
						else stopm = true;
						if (!stopp && blob_mat.at<uchar>(pos_yp, ptFar.x) == 255) {
							blob_mat.at<uchar>(pos_yp, ptFar.x) = 0;
							pos_yp++;
						}
						else stopp = true;
					}
					grassFireHandler gs = grassFireHandler();
					gs.setImage(&blob_mat);
					for (int y = 0; y < blob_mat.rows; y++) {
						for (int x = 0; x < blob_mat.cols; x++) {
							if (blob_mat.at<uchar>(y, x) == 255) {
								gs.setSeed(Point(x, y));
								grassResult blob1 = gs.runFromSeed(true);
								gs.reset();
								new_blobs.push_back(blob1.MBLOB);
							}
						}
					}
					if (new_blobs.size() > 1) {
						for (int i = 0; i < new_blobs[0].getPoints().size(); i++) {
							new_blobs[0].setPoint(i, Point(new_blobs[0].getPoints()[i].x + blob.getBBoxL().x, new_blobs[0].getPoints()[i].y + blob.getBBoxL().y));
						}
						for (int i = 0; i < new_blobs[1].getPoints().size(); i++) {
							new_blobs[1].setPoint(i, Point(new_blobs[1].getPoints()[i].x + blob.getBBoxL().x, new_blobs[1].getPoints()[i].y + blob.getBBoxL().y));
						}
						new_blobs[0].setBBoxL(Point(blob.getBBoxL().x + new_blobs[0].getBBoxL().x, blob.getBBoxL().y + new_blobs[0].getBBoxL().y));
						new_blobs[0].setBBoxH(Point(blob.getBBoxL().x + new_blobs[0].getBBoxH().x, blob.getBBoxL().y + new_blobs[0].getBBoxH().y));
						new_blobs[1].setBBoxL(Point(blob.getBBoxL().x + new_blobs[1].getBBoxL().x, blob.getBBoxL().y + new_blobs[1].getBBoxL().y));
						new_blobs[1].setBBoxH(Point(blob.getBBoxL().x + new_blobs[1].getBBoxH().x, blob.getBBoxL().y + new_blobs[1].getBBoxH().y));
						new_blobs[0].setCoM(Point(blob.getBBoxL().x + new_blobs[0].getCoM().x, blob.getBBoxL().y + new_blobs[0].getCoM().y));
						new_blobs[1].setCoM(Point(blob.getBBoxL().x + new_blobs[1].getCoM().x, blob.getBBoxL().y + new_blobs[1].getCoM().y));
						mblobs = { new_blobs[0],new_blobs[1] };
					}
				}
				else if (type == 1) {
					int pos_xm = ptFar.x - 1;
					int pos_xp = ptFar.x + 1;
					bool stopm = false;
					bool stopp = false;
					blob_mat.at<uchar>(ptFar.y, ptFar.x) = 0;
					for (int i = 0; i < blob.getBBox().width; i++) {
						if (stopp && stopm) break;
						if (pos_xm < 0) stopm = true;
						if (pos_xp > blob.getBBox().width-1) stopp = true;
						if (!stopm && blob_mat.at<uchar>(ptFar.y, pos_xm) == 255) {
							blob_mat.at<uchar>(ptFar.y,pos_xm) = 0;
							pos_xm--;
						}
						else stopm = true;
						if (!stopp && blob_mat.at<uchar>(ptFar.y, pos_xp) == 255) {
							blob_mat.at<uchar>(ptFar.y,pos_xp) = 0;
							pos_xp++;
						}
						else stopp = true;
					}
					grassFireHandler gs = grassFireHandler();
					gs.setImage(&blob_mat);
					vector<MBLOB> new_blobs;
					for (int y = 0; y < blob_mat.rows; y++) {
						for (int x = 0; x < blob_mat.cols; x++) {
							if (blob_mat.at<uchar>(y, x) == 255) {
								gs.setSeed(Point(x, y));
								grassResult blob1 = gs.runFromSeed(true);
								gs.reset();
								new_blobs.push_back(blob1.MBLOB);
							}
						}
					}
					if (new_blobs.size() > 1) {
						for (int i = 0; i < new_blobs[0].getPoints().size(); i++) {
							new_blobs[0].setPoint(i, Point(new_blobs[0].getPoints()[i].x + blob.getBBoxL().x, new_blobs[0].getPoints()[i].y + blob.getBBoxL().y));
						}
						for (int i = 0; i < new_blobs[1].getPoints().size(); i++) {
							new_blobs[1].setPoint(i, Point(new_blobs[1].getPoints()[i].x + blob.getBBoxL().x, new_blobs[1].getPoints()[i].y + blob.getBBoxL().y));
						}
						new_blobs[0].setBBoxL(Point(blob.getBBoxL().x, blob.getBBoxL().y));
						new_blobs[0].setBBoxH(Point(blob.getBBoxH().x, blob.getBBoxL().y + new_blobs[0].getBBoxH().y));
						new_blobs[1].setBBoxL(Point(blob.getBBoxL().x, blob.getBBoxL().y + new_blobs[1].getBBoxL().y));
						new_blobs[1].setBBoxH(Point(blob.getBBoxH().x, blob.getBBoxH().y));
						new_blobs[0].setCoM(Point((new_blobs[0].getBBoxL().x + ((float)(new_blobs[0].getBBoxH().x - new_blobs[0].getBBoxL().x)) / 2), (new_blobs[0].getBBoxL().y + ((float)(new_blobs[0].getBBoxH().y - new_blobs[0].getBBoxL().y)) / 2)));
						new_blobs[1].setCoM(Point((new_blobs[1].getBBoxL().x + ((float)(new_blobs[1].getBBoxH().x - new_blobs[1].getBBoxL().x)) / 2), (new_blobs[1].getBBoxL().y + ((float)(new_blobs[1].getBBoxH().y - new_blobs[1].getBBoxL().y)) / 2)));
						mblobs = { new_blobs[0],new_blobs[1] };
					}
				}
			}
		}
		else {
			mblobs = { blob };
		}
	}
	return mblobs;
}

/*MBLOB reunion(vector<MBLOB> mblobs, MBLOB blob, Mat bin, int norm) {
	MBLOB potential;
	potential.setBBoxL(blob.getBBoxL());
	potential.setBBoxH(Point(blob.getBBoxL().x + (int)(80 * norm / 3), blob.getBBoxL().y + 80 * norm));
	int count = 0;
	vector<MBLOB> counted = { blob };
	for (int i = potential.getBBoxL().y; i < potential.getBBoxH().y; i++) {
		for (int j = potential.getBBoxL().x; j < potential.getBBoxH().x; j++) {
			if (bin.at<uchar>(i, j) == 255) {
				potential.resize(count+1);
				potential.setPoint(count,Point(j, i));
				count++;
			}
		}
	}
	potential.setCoM(Point((int)((blob.getBBoxL().x + (int)(80 * norm / 6))), (int)((blob.getBBoxL().y + 80 * norm)) / 2));

	float pxratio = ((float)count) / (potential.getBBox().height*potential.getBBox().width);
	if (pxratio < 0.3f) {
		return potential.noBlob();
	}
	else if (pxratio > 0.3f) {
		return potential;
	}
}*/


void main(){
	//path p("cam/");
	//convert_to_RGB(p,".png");

	XmlWriter xml;

	Image im("cam/00001.png");

	//Image im("http://169.254.12.63/mjpg/video.mjpg");
	//im.getBG(200);
	im.setBG("bg.png");
	//Image im(1);
	//im.getBG();
	destroyAllWindows();
	//Mat out = imread("overlay.png",CV_LOAD_IMAGE_ANYCOLOR);
	Mat out = Mat::zeros(im.returnBG().size(),CV_8UC3);
	memoryTracker mem = memoryTracker();
	int counter = 0;
	bool cont = true;

	if (xml.open("C:\\Users\\Cedric\\Downloads\\Clean-20170307T121810Z-001\\Clean\\Clean1\\KFdetections.xml")) { //change this path

		xml.writeOpenTag("tracking");

		while (cont) {

			xml.writeStartElementTag("frame");
			ostringstream str;
			str << "num=\"" << counter + 1 << "\"";
			xml.writeAttribute(str.str());

			cout << counter << endl;
			counter++;
			Mat frame = im.getNextFrame();
			if (frame.empty())break; //break id video ends;
			//imshow("Frame", frame);
			Mat sub = im.subtractBG(frame);
			BinaryImage bin = im.binarize(sub, 10);
			//if(out.empty()) out = Mat::zeros(sub.size(), CV_8UC3);
			//bin.close(3, 10);
			bin.open(5);
			//bin.close(0, 12);
			Mat binImg = bin.get();
			//Canny(binImg, binImg, 0, 255);
			threshold(binImg, binImg, 30, 255, THRESH_BINARY);
			Mat origbin;
			binImg.copyTo(origbin);
			//imshow("bin", binImg);
			Mat grad_x, grad_y, abs_grad_x, abs_grad_y;
			Sobel(binImg, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
			Sobel(binImg, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
			convertScaleAbs(grad_x, abs_grad_x);
			convertScaleAbs(grad_y, abs_grad_y);

			//imshow("GRAD_X", abs_grad_x);
			//imshow("GRAD_Y", abs_grad_y);

			mem.cleanUp();
			//Start grassFire;
			grassFireHandler gs = grassFireHandler();
			gs.setImage(&binImg);
			Mat newbin;
			binImg.copyTo(newbin);
			vector<MBLOB> blobs;
			for (int y = 0; y < binImg.rows; y++) {
				for (int x = 0; x < binImg.cols; x++) {
					if (binImg.at<uchar>(y, x) == 255) {
						gs.setSeed(Point(x, y));
						grassResult blob = gs.runFromSeed(true);
						gs.reset();
						blobs.push_back(blob.MBLOB);
					}
				}
			}
			//have all blobs

			int width_type = 0;
			int height_type = 1;
			Mat test = Mat::zeros(binImg.size(), CV_8UC1);

			for (int i = 0; i < blobs.size(); i++) {

				float ratio = ((float)blobs[i].getBBox().height) / ((float)blobs[i].getBBox().width);
				float pxratio = ((float)blobs[i].getPoints().size()) / (blobs[i].getBBox().height*blobs[i].getBBox().width);
				//std::cout << ratio << std::endl;
				if (ratio < 1.5f) {
					MBLOB current = blobs[i];
					vector<MBLOB> newBlobs = defects(blobs[i], abs_grad_x, abs_grad_y, test, width_type);
					blobs.erase(blobs.begin() + i);
					for (int j = 0; j < newBlobs.size(); j++) {
						blobs.insert(blobs.begin() + i + j, newBlobs[j]);
					}
					i += newBlobs.size() - 1;
				}
				else if ((ratio > 2 && pxratio < 0.5f) || ratio > 3.5f) {
					MBLOB current = blobs[i];
					vector<MBLOB> newBlobs = defects(blobs[i], abs_grad_x, abs_grad_y, test, height_type);
					blobs.erase(blobs.begin() + i);
					for (int j = 0; j < newBlobs.size(); j++) {
						blobs.insert(blobs.begin() + i + j, newBlobs[j]);
					}
					i += newBlobs.size() - 1;
				}
			}

			img_defects(newbin);


			for (int i = 0; i < blobs.size(); i++) {

				if (blobs[i].getPoints().size() > 150) {
					for (int j = 0; j < blobs[i].getPoints().size(); j++) {
						test.at<uchar>(blobs[i].getPoints()[j].y, blobs[i].getPoints()[j].x) = 255;
					}
				}
				else {
					blobs.erase(blobs.begin() + i);
					i--;
				}

				/*MBLOB m = reunion(blobs, blobs[i], test, 1.2f);
				blobs.erase(blobs.begin() + i);
				if (!m.contains(Point(-1, -1))) {
				blobs.insert(blobs.begin() + i, m);
				i++;
				}
				i--;*/
			}


			for (int i = 0; i < blobs.size(); i++) {
				for (int j = 0; j < blobs[i].getPoints().size(); j++) {
					test.at<uchar>(blobs[i].getPoints()[j].y, blobs[i].getPoints()[j].x) = 255;
				}
				rectangle(test, blobs[i].getBBoxL(), blobs[i].getBBoxH(), Scalar(255, 255, 255), 1);
			}

			imshow("Blobs", test);

			//Get ROI
			vector<ROI> rois;
			for (int i = 0; i < mem.getMemSize(); i++) {
				rois.push_back(mem.getMem(i).getRoi());
			}
			//Have ROIs so make the table
			//if (rois.size() > 0 && blobs.size() > 0){
			/* BLOB ID Algorithm */
			vector<bool> assignedBlobs(blobs.size(), false);
			vector<bool> assignedRois(rois.size(), false); //We have not assigned anything yet

			vector<Priorities> roiPri; // Set up priorities
									   //Go through each ROI and find priorities
			for (int i = 0; i < rois.size(); i++) {
				Priorities pri;
				Point ROICenter = rois[i].getCenter();
				for (int j = 0; j < blobs.size(); j++) {
					Point BLOBCenter = blobs[j].getCoM();
					Priority blob;
					blob.id = j;
					blob.isInRoI = rois[i].isPtInROI(BLOBCenter);
					Point dist = BLOBCenter - ROICenter;
					blob.dist = sqrt(dist.x*dist.x + dist.y*dist.y);
					pri.addPriority(blob);
				}
				roiPri.push_back(pri);
			}
			for (int i = 0; i < roiPri.size(); i++) {
				roiPri[i].sortPriorities();
			}

			// Have priorities for ROIs
			/*			for (int i = 0; i < roiPri.size(); i++){
			cout << endl << endl << "ROI " << i << ":" << endl;
			rois[i].extractFromImage(origbin);
			//imshow("Roi" + to_string(i), rois[i].getImage());
			for (int i2 = 0; i2 < roiPri[i].size(); i2++){
			cout << "Priority " << i2 << ": BLOB " << roiPri[i].getPriority(i2).id << endl;

			}
			}*/
			//waitKey(0);
			//system("PAUSE");
			bool restart = true;
			while (restart) {
				restart = false;
				for (int i = 0; i < roiPri.size(); i++) {
					if (roiPri[i].size() == 0) continue;
					Priority currentFirst = roiPri[i].getPriority(0);
					for (int j = 0; j < roiPri.size(); j++) {
						if (roiPri[j].size() == 0) continue;
						if (j == i) continue;
						Priority otherFirst = roiPri[j].getPriority(0);
						if (currentFirst.id == otherFirst.id) {
							int toKeep;
							//cout << "Conflicting priorities \n";
							if (currentFirst.isInRoI != otherFirst.isInRoI) {
								if (currentFirst.isInRoI) toKeep = i; else toKeep = j;
							}
							else {
								//Minimalize distance
								if (roiPri[i].size() > 1 && roiPri[j].size() > 1) {
									float d1 = currentFirst.dist + roiPri[j].getPriority(1).dist;
									float d2 = roiPri[i].getPriority(1).dist + otherFirst.dist;
									toKeep = (d1 < d2) ? i : j;
								}
								else {
									toKeep = (currentFirst.dist < otherFirst.dist) ? i : j;
								}
							}
							//Resolve conflicts
							int toDelete = (toKeep == i) ? j : i;
							roiPri[toDelete].removePriority(0);
							if (toDelete == i) {
								restart = true;
								break;
							}
						}
					}
					if (restart) break;
				}
			}

			//Have assignments
			//Do assignents

			bool sw = false;

			for (int i = 0; i < roiPri.size(); i++) {
				if (roiPri[i].size() == 0) {
					// Roi lost
					mem.memDisappear(i);
					Point pred = mem.getMemROI(i).getCenter();
					Point est = mem.addMeasurementTo(i, pred, NULL, true);
					assignedRois[i] = true;
				}
				else {
					MBLOB blobToAdd = blobs[roiPri[i].getPriority(0).id];
					Point blobSize = blobToAdd.getBBoxH() - blobToAdd.getBBoxL();
					float thresh = 1.2f*sqrt(blobSize.x*blobSize.x + blobSize.y*blobSize.y);
					if (roiPri[i].getPriority(0).dist > thresh) continue;
					//Add blob to roi
					if (!sw) {
						sw = true;
						xml.writeCloseNewTag();
					}
					xml.writeStartElementNewTag("object");
					ostringstream str1;
					str1 << "ID=\"" << i << "\"";
					xml.writeAttribute(str1.str());
					xml.writeStartElementBox("box");
					ostringstream str2;
					str2 << "h=\"" << blobToAdd.getBBox().height << "\" w=\"" << blobToAdd.getBBox().width << "\" xb=\"" << blobToAdd.getBBoxL().x << "\" yb=\"" << blobToAdd.getBBoxL().y << "\"";
					xml.writeAttribute(str2.str());
					xml.writeEndElementBox();
					xml.writeEndElementTag();
					mem.memReappear(i);
					bool draw = mem.toDraw(i);
					Point prev = mem.getPrevPoint(i);
					Point est = mem.addMeasurementTo(i, blobToAdd.getCoM(), NULL, draw ? false : true); 
					assignedRois[i] = true;
					assignedBlobs[roiPri[i].getPriority(0).id] = true;
					//Draw
					if (draw)
						line(out, prev, est, mem.getColor(i), 1);
				}
			}
			for (int i = 0; i < blobs.size(); i++) {
				if (assignedBlobs[i]) continue;
				assignedBlobs[i] = true;
				mem.addNewMem(blobs[i].getCoM());

				//cout << "New BLOB" << endl;
			}

			/* END */
			//}
			mem.predict();
			//Add the rest to the end
			/*if (mem.getMemSize() == 0){
			for (int i = 0; i < blobs.size(); i++){
			mem.addNewMem(blobs[i].getCoM());
			}
			}
			else{*/
			imshow("line", out);
			namedWindow("line", WINDOW_NORMAL);
			if (roiPri.size() == 0) xml.writeEndElementNULLTag();
			else xml.writeEndElementTag();
			//resizeWindow("line", 800, 700);
			waitKey(20);
			//}//system("PAUSE");
			if (counter % 300 == 0) {
				imwrite("TEST.png", out);
			}
			if (waitKey(20) == 32) {
				imwrite("TEST.png", out);
				cont = false;
			}
			if (counter >= 3019) {
				imwrite("tracking_results.png", out);
				cont = false;
			}
		}
		xml.writeCloseTag();
		xml.close();
	}
	else {
		std::cout << "Error opening file.\n";
	}

	

	return;
}