//#include <pthread.h>
#include <sstream>
#include "functions.h"
#include "serialsom.h"

#define SEND				// 如需要串口发送，取消注释
#define DEBUG

const int Width = 800;		// 视频宽
const int Height = 600;		// 视频高

int MaxArea = 8000;			// 每个宫格轮廓的最大面积
int MinArea = 4000;			// 每个宫格轮廓的最小面积
int minGrayValue;			// 宫格内的最低灰度值，如果小于该值则认为宫格内没有内容（数字）
int ninxiTubeGrayValue;		// 数码管分割灰度阈值

VideoCapture cap;
VideoWriter writer;
Mat frame, gray_img, canny_img;
Mat element0, element1, element2;
Mat bMat, gMat, rMat;

Point2f srcPoints[4];		// 透视变换 srcPoints
Point2f dstPoints[4];		// 透视变换 dstPoints
int t1 = 100, t2 = 200;		// canny阈值
int password[5];			// 密码区（数码管）的 5 个数字
int passwordLast[5];		// 上一帧检识别出的密码，和当前帧做对比，如果有超过 3 个数字改变则认为密码改变
int nineNumber[9];			// 九宫格区的 9 个数字
int nineNumberLast[9];		// 上一帧检识别出的九个数字，和当前帧做对比，如果有超过 5 个数字改变则认为九宫格数字改变
int currentNumberCount = 0;	// 当前目标是数码管五位数中第几个，0~4
int errorCount;				// 识别错误数字的个数
float neighborDistance[9];	// kNN 识别每个数字与最近邻居的距离，值越小说明是该值的可能性越大

Rect passwordRect(0, 0, 200, 60);	// 九宫格区的 Rect
Mat pw_gray, pw_bin;				// 密码区（数码管）的灰度图和二值图
bool foundNixieTubeArea = false;	// 是否发现数码管区
bool isEmpty = false;				// 宫格中是否有数字
bool passwordChanged = true;		// 密码区是否改变	
bool nineNumberChanged = true;		// 九宫格区是否改变
bool getNinxiTubeGrayValue = false;

bool distinguishBuff = true;
int bigBuff = 0;					// 九宫格是大符还是小符，0——小符，1——大符，-1——不确定
int frameCount = 0;
int recordVideo = 0;				// 是否录像
int videoName;						// 录像文件名
int findNineRect = 0;				// 0 - 未发现九宫格，20 - 发现九宫格
int key;

void* capFrameThread(void *arg);

int main(int argc, char** argv)
{

#ifdef SEND	
	// 初始化串口
	Serialport Serialport1("/dev/ttyTHS0");
	int fd = Serialport1.open_port("/dev/ttyTHS0");
	while (fd < 0) {
		sleep(1);
		fd = Serialport1.open_port("/dev/ttyTHS0");
	}
	Serialport1.set_opt(115200, 8, 'N', 1);
#endif
	
	// 读取配置文件
	FileStorage fs("config.xml", FileStorage::READ);

	string filename;
	fs["filename"] >> filename;
	fs["minGrayValue"] >> minGrayValue;
	fs["ninxiTubeGrayValue"] >> ninxiTubeGrayValue;
	fs["t1"] >> t1;
	fs["t2"] >> t2;
	fs["recordVideo"] >> recordVideo;

	fs.release();
	
	if (recordVideo) {
		FileStorage fs1("record.xml", FileStorage::READ);
		fs1["videoName"] >> videoName;
		fs1.release();
		stringstream ss;
		ss << videoName;
		string s = ss.str();
		int fourcc = CV_FOURCC('M', 'J', 'P', 'G');
		writer.open(s + ".avi", fourcc, 25.0, Size(Width, Height));
		videoName++;
		FileStorage fs2("record.xml", FileStorage::WRITE);
		fs2 << "videoName" << videoName;
		fs2.release();
	}
	
	//cap.open(filename);
	// 打开摄像头
 	cap.open(0);
	while (!cap.isOpened()) {
		sleep(1);
		cap.open(0);
	}
	cap.set(CAP_PROP_FRAME_WIDTH, Width);
 	cap.set(CAP_PROP_FRAME_HEIGHT, Height);

	// 开启读取视频帧的线程
/*	pthread_t id;
	int ret = pthread_create(&id, NULL, capFrameThread, NULL);
	if (!ret) {
		cout << "open thread to capture frame: success" << endl;
	} else {
		cout << "open thread to capture frame: failed" << endl;
	}
*/
	element0 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	element2 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

	bMat.create(Size(Width, Height), CV_8UC1);
	gMat.create(Size(Width, Height), CV_8UC1);
	rMat.create(Size(Width, Height), CV_8UC1);
	Mat chan[3] = { bMat, gMat, rMat };
	
	dstPoints[0] = Point2f(0, 0);
	dstPoints[1] = Point2f(40, 0);
	dstPoints[2] = Point2f(40, 40);
	dstPoints[3] = Point2f(0, 40);
	
	Mat warpMat(2, 4, CV_32FC1);
	Mat dstImage(40, 40, CV_8UC1);
	
	/************************************************************************/
	/*                     初始化KNearest数字识别（手写体）                   */
	/************************************************************************/
	FileStorage fsClassifications("classifications.xml", FileStorage::READ);  // 读取 classifications.xml 分类文件

	if (!fsClassifications.isOpened()) {
		cout << "ERROR, cannot open classifications.xml\n\n";
		//system("pause");
		return 0;
	}

	Mat matClassificationInts;
	fsClassifications["classifications"] >> matClassificationInts;  // 把 classifications.xml 中的 classifications 读取进Mat变量
	fsClassifications.release();  // 关闭文件

	FileStorage fsTrainingImages("images.xml", FileStorage::READ);  // 打开训练图片文件

	if (!fsTrainingImages.isOpened()) {
		cout << "ERROR, cannot open images.xml\n\n";
		//system("pause");
		return 0;
	}

	// 读取训练图片数据（从images.xml中）
	Mat matTrainingImagesAsFlattenedFloats;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages["images"] >> matTrainingImagesAsFlattenedFloats;  // 把 images.xml 中的 images 读取进Mat变量
	fsTrainingImages.release();

	// 训练
	Ptr<ml::KNearest> kNearest(ml::KNearest::create());  // 实例化 kNearest 对象

	// 最终调用train函数
	kNearest->train(matTrainingImagesAsFlattenedFloats, ml::ROW_SAMPLE, matClassificationInts);

	// 计算目标图像的HOG特征（方向梯度直方图特征）代替之前用的灰度特征
	HOGDescriptor *hog = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors;

	/************************************************************************/
	/*                  初始化KNearest数字识别（数码管）                      */
	/************************************************************************/
	FileStorage fsClassifications1("classifications1.xml", FileStorage::READ);
	if (!fsClassifications1.isOpened()) {
		cout << "ERROR, cannot open classifications1.xml\n\n";
		return 0;
	}
	Mat matClassificationInts1;
	fsClassifications1["classifications"] >> matClassificationInts1;
	fsClassifications1.release();
	FileStorage fsTrainingImages1("images1.xml", FileStorage::READ);
	if (!fsTrainingImages1.isOpened()) {
		cout << "ERROR, cannot open images.xml\n\n";
		return 0;
	}
	Mat matTrainingImagesAsFlattenedFloats1;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages1["images"] >> matTrainingImagesAsFlattenedFloats1;  // 把 images.xml 中的 images 读取进Mat变量
	fsTrainingImages1.release();
	Ptr<ml::KNearest> kNearest1(ml::KNearest::create());
	kNearest1->train(matTrainingImagesAsFlattenedFloats1, ml::ROW_SAMPLE, matClassificationInts1);
	HOGDescriptor *hog1 = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors1;

	/************************************************************************/
	/*					    初始化 KNearest 小符检测	                    */
	/************************************************************************/
	FileStorage fsClassifications2("classifications2.xml", FileStorage::READ);
	if (!fsClassifications2.isOpened()) {
		cout << "ERROR, cannot open classifications2.xml\n\n";
		return 0;
	}
	Mat matClassificationInts2;
	fsClassifications2["classifications"] >> matClassificationInts2;
	fsClassifications2.release();
	FileStorage fsTrainingImages2("images2.xml", FileStorage::READ);
	if (!fsTrainingImages2.isOpened()) {
		cout << "ERROR, cannot open images2.xml\n\n";
		return 0;
	}
	Mat matTrainingImagesAsFlattenedFloats2;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages2["images"] >> matTrainingImagesAsFlattenedFloats2;  // 把 images.xml 中的 images 读取进Mat变量
	fsTrainingImages2.release();
	Ptr<ml::KNearest> kNearest2(ml::KNearest::create());
	kNearest2->train(matTrainingImagesAsFlattenedFloats2, ml::ROW_SAMPLE, matClassificationInts2);
	HOGDescriptor *hog2 = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors2;
	
	/************************************************************************/
	/*                         开始检测每一帧图像                             */
	/************************************************************************/
	while (true) {
		
		cap >> frame;

		if (frame.empty()) {
#ifdef DEBUG		
			break;
#else
			continue;
#endif
		}
		
		if (recordVideo)
			writer.write(frame);
		
		cvtColor(frame, gray_img, COLOR_BGR2GRAY);
		Canny(gray_img, canny_img, t1, t2);
		dilate(canny_img, canny_img, element2);	// 膨胀

		if (bigBuff)
			split(frame, chan);
		
#ifdef DEBUG
		key = (waitKey(10) & 255);
		if (key == 32) waitKey(0);
		else if (key == 27) break;
#endif

		// 寻找所有轮廓
		vector<vector<Point> > contours0;		// 所有轮廓
		findContours(canny_img, contours0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// 先用面积约束，面积在指定范围内的轮廓的数量不满足要求时，进入下一帧
		vector<vector<Point> > contours1;		// 面积在指定范围内的轮廓（九宫格区）
		for (size_t i = 0; i < contours0.size(); i++) {
			double area = contourArea(contours0[i]);
			if (area > MinArea && area < MaxArea) {
				contours1.push_back(contours0[i]);
			}
		}

	 	if (contours1.size() < 9) {
			// 指定面积范围内的轮廓较少，有可能只是漏检测，这里做一些约束
			if (frameCount == 3 || frameCount == 30)
				frameCount = 0;
			if (frameCount < 10 && !distinguishBuff)
				frameCount += 2;
			if (frameCount == 10)	// 每次+2，连续5帧后，distinguishBuff = true，开始重新分辨小符和大符
				distinguishBuff = true;
			
			if (findNineRect > 0)
				findNineRect--;

#ifdef SEND				
			if (findNineRect == 0)
				Serialport1.usart3_send(static_cast<uint8_t>(255));
#endif

#ifdef DEBUG
			cout << "contours in specified range are not enough" << endl;
#endif
			continue;
		}

		// 上面得到的轮廓可能有凸缺陷（如果数字的笔画延伸到了轮廓的边缘）
		// 下面需要寻找凸包，绘制包含凸缺陷的较规范的矩形
		vector<vector<Point> > contours2;			// 面积在指定范围内的轮廓（九宫格区_）
		for (size_t i = 0; i < contours1.size(); i++) {
			vector<int> hull;						// 找到的凸包（其实是轮廓最外层点的索引）
			convexHull(contours1[i], hull, true);	// 寻找点集（轮廓）的凸包

			vector<Point> tempContour;				// 用凸包制作点集（轮廓），用于计算面积
			for (size_t j = 0; j < hull.size(); j++)
				tempContour.push_back(contours1[i][hull[j]]);
			double tempArea = contourArea(tempContour);
			if (tempArea < 4000 || tempArea > 8000)	// 面积约束
				continue;

			vector<Point> tempContour2;
			approxPolyDP(tempContour, tempContour2, 30, true);
			if (tempContour2.size() != 4)			// 逼近多边形，剔除非四边形
				continue;

			contours2.push_back(tempContour2);
		}

#ifdef DEBUG		
		Mat contours_Mat(Height, Width, CV_8UC1, Scalar(0));
		drawContours(contours_Mat, contours2, -1, Scalar(255), -1, LINE_AA);
		imshow("contours_Mat", contours_Mat);
#endif
		
		// 对面积在指定范围内的轮廓取最小包围矩形（九宫格区）
		vector<RotatedRect> contours_rotatedRect;
		for (int i = 0; i < contours2.size(); i++)
			contours_rotatedRect.push_back(minAreaRect(contours2[i]));

		// 如果旋转矩形的数量大于9，则做一些约束，角度，长宽比
		if (contours_rotatedRect.size() > 9) {
			vector<RotatedRect> contours_rotatedRect_tmp;
			for (size_t i = 0; i < contours_rotatedRect.size(); i++) {
				bool b1 = false, b2 = false, b3 = false;

				float tmp_float = contours_rotatedRect[i].size.width / contours_rotatedRect[i].size.height;
				float tmp_area = contours_rotatedRect[i].size.width * contours_rotatedRect[i].size.height;
				if (tmp_float < 1) {
					tmp_float = 1.0 / tmp_float;
					swap(contours_rotatedRect[i].size.height, contours_rotatedRect[i].size.width);
					contours_rotatedRect[i].angle = contours_rotatedRect[i].angle + 90;
				}

				if (contours_rotatedRect[i].angle > -5 && contours_rotatedRect[i].angle < 5) {
					b1 = true;
				}

				if (tmp_float > 1.40 && tmp_float < 2.0) {
					b2 = true;
				}

				if (tmp_area > MinArea && tmp_area < (MaxArea + 200)) {
					b3 = true;
				}

				if (b1 && b2)
					contours_rotatedRect_tmp.push_back(contours_rotatedRect[i]);
			}
			contours_rotatedRect.clear();
			contours_rotatedRect = contours_rotatedRect_tmp;
		}

		// 找到了九个宫格的位置，提取对应ROI，用于识别
		vector<Mat> nineRect_mat(9);	// 经过阈值分割和膨胀处理的九宫格（数字）的Mat
		isEmpty = false;
		if (contours_rotatedRect.size() == 9) {
			if (findNineRect < 20)
				findNineRect++;
			
			sortRotatedRect(contours_rotatedRect);	// 把9个旋转矩形按照顺序排序
			// 对于每个旋转矩形，找到其四个顶点，使用仿射变换将其变换为正矩形
			for (int i = 0; i < 9; i++) {
				vector<Point2f> p(4);
				contours_rotatedRect[i].points(p.data());
				sortPoints(p);

				srcPoints[0] = p[0] + Point2f(10, 6);		// 左上
				srcPoints[1] = p[1] + Point2f(-10, 6);		// 右上
				srcPoints[2] = p[2] + Point2f(-10, -4);		// 右下
				srcPoints[3] = p[3] + Point2f(10, -4);		// 左下

				// 利用第一个宫格的右上角和第三个宫格的左上角来定位密码区（数码管）的位置，得到相应的Rect
				if (i == 0) {
					passwordRect.x = p[1].x - 20;
					passwordRect.y = p[1].y - 80;
				}
				else if (i == 2) {
					passwordRect.width = p[0].x + 15 - passwordRect.x;
					passwordRect.height = p[0].y - 20 - passwordRect.y;
				}

				if (passwordRect.x < 0 || (passwordRect.x + passwordRect.width) > Width ||
					passwordRect.y < 0 || (passwordRect.y + passwordRect.height) > Height ||
					passwordRect.width < 10 || passwordRect.height < 10)
					foundNixieTubeArea = false;
				else
					foundNixieTubeArea = true;
#ifdef DEBUG
				for (int j = 0; j < 4; j++)
					line(frame, srcPoints[j], srcPoints[(j + 1) % 4], Scalar(204, 122, 0), 2, LINE_AA);
#endif

				// 透视变换，将密码区变换成 40*40 的Mat
				warpMat = getPerspectiveTransform(srcPoints, dstPoints);
				warpPerspective(gray_img, dstImage, warpMat, dstImage.size());

				if (!getNinxiTubeGrayValue) {
					Mat matTemp;
					double thresh = threshold(dstImage, matTemp, 0, 255, THRESH_OTSU);
					if (thresh > 40 && thresh < 200) {
						minGrayValue = thresh;
						getNinxiTubeGrayValue = true;
					}
					//cout <<  minGrayValue << endl;
				}
				
				// 九宫格内的数字在两次变换之间有短暂时间没有内容（空白）
				// 这里通过最低灰度值来判断是否存在数字
				if (min_mat(dstImage) > minGrayValue) {
					cout << " no number in cells" << endl;
					isEmpty = true;
					break;
				}

				threshold(dstImage, dstImage, 0, 255, THRESH_OTSU);
				threshold(dstImage, dstImage, 50, 255, THRESH_BINARY_INV);
				dilate(dstImage, dstImage, element2);	// 膨胀
				if (bigBuff == 1)
					deskew(dstImage);					// 抗扭斜处理
				vector<vector<Point> > tempContour;
				findContours(dstImage, tempContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
				Rect rect;
				if (tempContour.size() > 1) {
					double tempArea = 0;
					for (int j = 0; j < tempContour.size(); j++) {
						if (tempArea < contourArea(tempContour[j])) {
							rect = boundingRect(tempContour[j]);
							tempArea = contourArea(tempContour[j]);
						}
					}
				}
				else if (tempContour.size() == 1) {
					rect = boundingRect(tempContour[0]);
				}
				else {
					isEmpty = true;
					break;
				}
				drawContours(dstImage, tempContour, -1, Scalar(255), -1);
				resize(dstImage(rect), nineRect_mat[i], Size(40, 40));
				if (bigBuff == 1)
					blur(nineRect_mat[i], nineRect_mat[i], Size(3, 3));
			}
		}
		else {
#ifdef DEBUG
			imshow("frame", frame);
#endif
			if (findNineRect > 0)
				findNineRect--;

#ifdef SEND				
			if (findNineRect == 0)
				Serialport1.usart3_send(static_cast<uint8_t>(255));
#endif
			continue;
		}

		// 如果第一个宫格中没有数字，则跳过该帧
		if (isEmpty)
			continue;
		
		for (int i = 0; i < 9; i++) {
			hog2->compute(nineRect_mat[i], descriptors2);
			Mat matROIFlattenedFloat(1, (int)(descriptors2.size()), CV_32FC1, descriptors2.data());
			Mat matCurrentChar(0, 0, CV_32F);
			Mat m1(0, 0, CV_32F);
			Mat m2(0, 0, CV_32F);
			kNearest2->findNearest(matROIFlattenedFloat, 1, matCurrentChar, m1, m2);
			nineNumber[i] = (int)matCurrentChar.at<float>(0, 0);	// 保存九宫格区的九个数字
			neighborDistance[i] = m2.at<float>(0, 0);
		}

		// 是否需要区分小符和大符
		if (distinguishBuff) {
			int distanceCount = 0;
			for (int i = 0; i < 9; i++) {
				if (neighborDistance[i] < 6)
					distanceCount--;
				else if (neighborDistance[i] > 10)
					distanceCount++;
			}

			if (distanceCount < -4) {
				bigBuff = 0;
				frameCount++;
			}
			else if (distanceCount > 4) {
				bigBuff = 1;
				frameCount += 10;
			}
			else {
				bigBuff = -1;
				frameCount = 0;
#ifdef DEBUG					
				imshow("frame", frame);
#endif	
				continue;
			}

			if (frameCount == 3 || frameCount == 30)
				distinguishBuff = false;
			else
				continue;
		}

		// 检测到小符之后发送信号
		if (!distinguishBuff && bigBuff == 0) {
			for (int i = 0; i < 9; i++) {
				if (nineNumber[i]) {
#ifdef DEBUG		
					cout << "target --> " << i + 1 << endl;
					circle(frame, contours_rotatedRect[i].center, 5, Scalar(0, 0, 255), 2, LINE_AA);
#endif
#ifdef SEND
					Serialport1.usart3_send(static_cast<uint8_t>(i + 1));
#endif					
					break;
				}
			}
#ifdef DEBUG			
			imshow("frame", frame);
#endif			
			continue;
		}

		/************************************************************************/
		/*                         kNN识别九宫格区数字                          */
		/************************************************************************/
		for (int i = 0; i < 9; i++) {
			hog->compute(nineRect_mat[i], descriptors);
			Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());
			Mat matCurrentChar(0, 0, CV_32F);  // findNearest的结果保存在这里
			Mat m1(0, 0, CV_32F);
			Mat m2(0, 0, CV_32F);
			kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar, m1, m2);
			nineNumber[i] = (int)matCurrentChar.at<float>(0, 0);	// 保存九宫格区的九个数字
			neighborDistance[i] = m2.at<float>(0, 0);
		}
		
		/* 判断九宫格的识别结果，如果出现有两个数字相同的情况，则说明有一个数字识别错误
		 * 首先找出有多少个数字识别错误
		 */
		errorCount = 0;
		vector<int> errorPair;
		for (int i = 0; i < 8; i++) {
			for (int j = i + 1; j < 9; j++) {
				if (nineNumber[i] == nineNumber[j]) {
					errorCount++;
					errorPair.push_back(i);
					errorPair.push_back(j);
#ifdef DEBUG					
					//cout << i + 1 << " " << nineNumber[i] << " " << neighborDistance[i] << endl;
					//cout << j + 1 << " " << nineNumber[j] << " " << neighborDistance[j] << endl;
#endif
				}
			}
		}

		if (errorCount == 1) {
			if (neighborDistance[0] < 3) {
				nineNumber[errorPair[1]] = 0;
				int sum = 0;
				for (int k = 0; k < 9; k++)
					sum += nineNumber[k];
				nineNumber[errorPair[1]] = 45 - sum;
				goto HERE;
			}
			else if (neighborDistance[1] < 3) {
				nineNumber[errorPair[0]] = 0;
				int sum = 0;
				for (int k = 0; k < 9; k++)
					sum += nineNumber[k];
				nineNumber[errorPair[0]] = 45 - sum;
				goto HERE;
			}
			
			/* 如果只有一个数字识别错误，再次使用 kNearest->findNearest 寻找 3 个近邻
			 * 如果其中一个数字的 3 个近邻为同一个值，则认为该值正确，另一个值错误
			 */
			for (int i = 0; i < 2; i++) {
				hog->compute(nineRect_mat[errorPair[i]], descriptors);
				Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());
				Mat matCurrentChar(0, 0, CV_32F);  // findNearest的结果保存在这里
				Mat m1(0, 0, CV_32F);
				Mat m2(0, 0, CV_32F);
				kNearest->findNearest(matROIFlattenedFloat, 3, matCurrentChar, m1, m2);

				int as[3] = { (int)m1.at<float>(0, 0), (int)m1.at<float>(0, 1), (int)m1.at<float>(0, 2) };
				if (as[0] == as[1] && as[1] == as[2]) {
					nineNumber[errorPair[1 - i]] = 0;
					int sum = 0;
					for (int k = 0; k < 9; k++)
						sum += nineNumber[k];
					nineNumber[errorPair[1 - i]] = 45 - sum;
					goto HERE;
				}
			}

			/* 如果 3 个近邻不都一样，比较这两个数字属于识别值得可能性
			 * 如果 neighborDistance[j] 的值更大，说明 nineNumber[j] 识别错误
			 * 先将其赋值为 0，然后计算数组 nineNumber 每个值得和 sum
			 * 最后用 45 - sum 即为缺失的值（45是1~9的和）
			 */
			if (neighborDistance[errorPair[0]] < neighborDistance[errorPair[1]]) {
				nineNumber[errorPair[1]] = 0;
				int sum = 0;
				for (int k = 0; k < 9; k++)
					sum += nineNumber[k];
				nineNumber[errorPair[1]] = 45 - sum;
			}
			else {
				nineNumber[errorPair[0]] = 0;
				int sum = 0;
				for (int k = 0; k < 9; k++)
					sum += nineNumber[k];
				nineNumber[errorPair[0]] = 45 - sum;
			}
		}
		else if (errorCount == 2) {
			/* 如果有两个数字识别错误，首先找到识别错误的两个数字的编号，再找到数组中缺失的两个数
			 * 然后再次调用 kNearest->findNearest() ，不过这次将 k 值设为 3，找到 三个最近的邻居
			 * 然后分析这三个邻居中是否有那两个缺失的值，如果有则把两个识别错误的数字和缺失的两个数
			 * 对应起来
			 */
			int a1[2];	// 两个识别错误的数字的编号
			int b1[2];	// 未识别出的两个数字

			if (neighborDistance[errorPair[0]] < neighborDistance[errorPair[1]])
				a1[0] = errorPair[1];
			else
				a1[0] = errorPair[0];

			if (neighborDistance[errorPair[2]] < neighborDistance[errorPair[3]])
				a1[1] = errorPair[3];
			else
				a1[1] = errorPair[2];

			findMissingNumber(nineNumber, b1, 2);

			hog->compute(nineRect_mat[a1[0]], descriptors);
			Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());
			Mat matCurrentChar(0, 0, CV_32F);  // findNearest的结果保存在这里
			Mat m1(0, 0, CV_32F);
			Mat m2(0, 0, CV_32F);
			kNearest->findNearest(matROIFlattenedFloat, 3, matCurrentChar, m1, m2);

			for (int i = 0; i < 3; i++) {
				for (int j = 0; j < 2; j++) {
					if ((int)m1.at<float>(0, i) == b1[j]) {
						nineNumber[a1[0]] = b1[j];
						nineNumber[a1[1]] = b1[1-j];
						goto HERE;
					}
				}
			}

			hog->compute(nineRect_mat[a1[1]], descriptors);
			kNearest->findNearest(matROIFlattenedFloat, 3, matCurrentChar, m1, m2);

			for (int i = 1; i < 3; i++) {
				for (int j = 0; j < 2; j++) {
					if ((int)m1.at<float>(0, i) == b1[j]) {
						nineNumber[a1[1]] = b1[j];
						nineNumber[a1[0]] = b1[1 - j];
						goto HERE;
					}
				}
			}
		}

	HERE:

#ifdef DEBUG
		Mat nineNumberMat(120, 120, CV_8UC1);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				nineRect_mat[j + i * 3].copyTo(nineNumberMat(Rect(j * 40, i * 40, 40, 40)));
		imshow("nineNumberMat", nineNumberMat);
#endif
		
		//if (!foundNixieTubeArea)
		//	continue;
	
		pw_gray = rMat(passwordRect).clone();
		ninxiTubeGrayValue = calcNixietubeThreshold(pw_gray);

		// 由数码管区的 passwordRect 得到相应的ROI，并做一些预处理
		threshold(pw_gray, pw_bin, ninxiTubeGrayValue, 255, THRESH_BINARY);
		erode(pw_bin, pw_bin, element2);		// 腐蚀
		dilate(pw_bin, pw_bin, element1);		// 膨胀
		connectClosedPoint(pw_bin);

#ifdef DEBUG
		imshow("frame", frame);
		imshow("password", pw_bin);
		key = (waitKey(1) & 255);
		if (!foundNixieTubeArea) {
			continue;
		}
#endif

		Mat pw_bin_ = pw_bin.clone();
		vector<vector<Point> > ninxiTubeAreaContour;
		findContours(pw_bin_, ninxiTubeAreaContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		vector<Rect> ninxiTubeNumbRect;
		for (size_t i = 0; i < ninxiTubeAreaContour.size(); i++) {
			Rect tmpRect = boundingRect(ninxiTubeAreaContour[i]);
			int tmpHeight = MAX(tmpRect.width, tmpRect.height);
			if (tmpHeight > 22)
				ninxiTubeNumbRect.push_back(tmpRect);
		}

		if (ninxiTubeNumbRect.size() == 5) {
			sortRect(ninxiTubeNumbRect);
		}
		else {
#ifdef DEBUG
			cout << "Nixietube Threshold = " << ninxiTubeGrayValue << endl;
			cout << "ninxiTube ERROR, ninxiTubeNumbRect.size() = " << ninxiTubeNumbRect.size() << endl;
#endif
			continue;
		}
		
		/************************************************************************/
		/*                     kNN识别密码区（数码管）数字                        */
		/************************************************************************/
		for (int i = 0; i < 5; i++) {
			Mat matROI = pw_bin(ninxiTubeNumbRect[i]);
			resize(matROI, matROI, Size(40, 40));
			hog1->compute(matROI, descriptors1);
			Mat matROIFlattenedFloat(1, (int)(descriptors1.size()), CV_32FC1, descriptors1.data());
			Mat matCurrentChar(0, 0, CV_32F);
			kNearest1->findNearest(matROIFlattenedFloat, 1, matCurrentChar);
			password[i] = (int)matCurrentChar.at<float>(0, 0);		// 保存密码区的五个数字
		}
		
		// 判断数码管区域是否改变
		int changedNum = 0;
		for (int i = 0; i < 5; i++)
			if (password[i] != passwordLast[i])
				changedNum++;
		
		// 如果有两个（或更多）数码管显示的数字变化，则认为密码改变，当前目标为第一个数码管显示的数字
		if (changedNum > 1) {
			currentNumberCount = 0;
			
			for (int i = 0; i < 5; i++)
				passwordLast[i] = password[i];
		}

		// 判断九宫格区域是否改变
		changedNum = 0;
		for (int i = 0; i < 9; i++)
			if (nineNumber[i] != nineNumberLast[i])
				changedNum++;
		
		// 九宫格区已经改变，则查找当前目标数码管的数字在九宫格中的位置，然后目标数码管向右移动一个
		if (changedNum > 3) {
			for (int i = 0; i < 9; i++) {
				if (password[currentNumberCount] == nineNumber[i]) {
#ifdef DEBUG
					circle(frame, contours_rotatedRect[i].center, 5, Scalar(0, 0, 255), 2, LINE_AA);
					cout << "targetNum : " << currentNumberCount + 1 << "-->" << i + 1 << endl;
#endif
#ifdef SEND	
					Serialport1.usart3_send(static_cast<uint8_t>(i + 1));
#endif					
					if (currentNumberCount < 4)
						currentNumberCount++;
					break;
				}
			}
			
			for (int i = 0; i < 9; i++)
				nineNumberLast[i] = nineNumber[i];
		}
		
		getNinxiTubeGrayValue = false;
		
#ifdef DEBUG
		imshow("frame", frame);
		imshow("password", pw_bin);
#endif
		
		// 打印输出密码区和九宫格区的数字
#ifdef DEBUG

		for (int i = 0; i < 5; i++)
			cout << password[i];
		cout << "-->";

		for (int i = 0; i < 9; i++)
			cout << nineNumber[i];
		cout << endl << endl;

/*		key = (waitKey(1) & 255);
		if (key == 32)
			waitKey(0);
		else if (key == 27)
			break;*/
#endif
	}

	cap.release();
	writer.release();
	return 0;
}

void* capFrameThread(void *arg) {
	while(true) {
		if (cap.isOpened()) {
			cap >> frame;
			if (recordVideo)
				writer.write(frame);
		}
	}
    return NULL;
}

/*
double time0 = static_cast<double>(getTickCount());
time0 = ((double)getTickCount() - time0) / getTickFrequency();
cout << "用时" << time0 * 1000 << "毫秒" << endl;
*/
