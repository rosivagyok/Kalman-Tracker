#include <opencv2\core\core.hpp>	
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\video\video.hpp>
#include <iostream>

#define drawCross( center, color, d , img)                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 1, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 1, CV_AA, 0 )

#define drawBBOx(box,color,img)\
	rectangle(img,box,color)

using namespace std;
using namespace cv;

class BLOB{
private:
	vector<Point> points;
	Point CoM;
	Point BBoxL, BBoxH;
public:
	BLOB(){
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
		return this->CoM * ((double)1/this->points.size());
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

};
struct grassResult{
	Mat img;
	BLOB blob;
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
		this->reset();
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
	void reset(){
		this->offsetX = 0;
		this->offsetY = 0;
		this->img = NULL;
		this->toCheck.clear();
	}
	bool addPxToBl(BLOB &blob, Point bp){
		if (!blob.contains(bp)){
			blob.add(bp);
			this->toCheck.push_back(bp);
			return true;
		}
		return false;
	}
	grassResult runFromSeed(bool burn = false){
		Mat img = *this->img;
		BLOB blob;
		grassResult ret;
		if (img.at<uchar>(this->offsetY, this->offsetX) == 255){
			Point bp = Point(this->offsetX, this->offsetY);
			this->addPxToBl(blob, bp);
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
				if (img.at<uchar>(n) == 255) this->addPxToBl(blob, n);
				if (burn) img.at<uchar>(n) = 0;
			}
			if (ne.x >= 0 && ne.x < img.cols && ne.y >= 0 && ne.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(ne) == 255) this->addPxToBl(blob, ne);
				if (burn) img.at<uchar>(ne) = 0;
			}
			if (nw.x >= 0 && nw.x < img.cols && nw.y >= 0 && nw.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(nw) == 255) this->addPxToBl(blob, nw);
				if (burn) img.at<uchar>(nw) = 0;
			}
			if (e.x >= 0 && e.x < img.cols && e.y >= 0 && e.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(e) == 255) this->addPxToBl(blob, e);
				if (burn) img.at<uchar>(e) = 0;
			}
			if (w.x >= 0 && w.x < img.cols && w.y >= 0 && w.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(w) == 255) this->addPxToBl(blob, w);
				if (burn) img.at<uchar>(w) = 0;
			}
			if (sw.x >= 0 && sw.x < img.cols && sw.y >= 0 && sw.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(sw) == 255) this->addPxToBl(blob, sw);
				if (burn) img.at<uchar>(sw) = 0;
			}
			if (se.x >= 0 && se.x < img.cols && se.y >= 0 && se.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(se) == 255) this->addPxToBl(blob, se);
				if (burn) img.at<uchar>(se) = 0;
			}
			if (s.x >= 0 && s.x < img.cols && s.y >= 0 && s.y < img.rows){ //If point is in the Image
				if (img.at<uchar>(s) == 255) this->addPxToBl(blob, s);
				if (burn) img.at<uchar>(s) = 0;
			}
			this->toCheck.erase(this->toCheck.begin());
		}
		ret.blob = blob;
		ret.img = img;
		return ret;
	}
};

/*void main(){
	Mat img = imread("lol.png", CV_LOAD_IMAGE_GRAYSCALE);
	cout << img.cols << endl << img.rows << endl << (int)img.at<uchar>(Point(580, 181)) << endl;
	grassFireHandler gs = grassFireHandler();
	gs.setImage(&img);
	Point seedp;
	for (int y = 0; y < img.rows; y++){
		bool bme = false; // Break Me boolean
		for (int x = 0; x < img.cols; x++){
			//find first non-black pixel
			seedp = Point(x, y);
			if (img.at<uchar>(seedp) == 255){
				bme = true;
				break;
			}
		}
		if (bme) break;
	}
	gs.setSeed(seedp);
	grassResult a = gs.runFromSeed(false);
	drawCross(a.blob.getCoM(), Scalar(200), 5, img);
	drawBBOx(a.blob.getBBox(), Scalar(200), img);
	imshow("Res", img);
	waitKey(0);
}*/