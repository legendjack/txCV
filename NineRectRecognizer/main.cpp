#include "functions.h"
#include "getConfig.h"
#include "serialsom.h"

#define Width 640
#define Height 480
#define MinArea 1400
#define MaxArea 2000
#define DEBUG

map<string, string> config;
int thresh = 100;
bool first = true;
Mat frame, gray_img, canny_img;
Mat element0;
int t1 = 200, t2 = 250;			// canny阈值
int key;						// waitKey()返回值
int password[5];				// 密码区的5个数
int password_int = 0;			// 密码区前两个数组成的两位数，如果改变则认为密码变化
bool password_changed = true;	// 密码区是否改变	
bool nineNumber_changed = true; // 九宫格区是否改变
int nineNumber[9];				// 九宫格区的9个数
int nineNumber_int = 0;			// 九宫格第一行的三个数组成的三位数，如果变化则认为九宫格区改变
int count_ = 0;					// 计数用，当前目标是密码区第count_个数
int targetNum[3] = { 0,0,1 };	// 缓存3帧的将要发送的数据，如果三个数相等，则发送，否则认为有一帧误检，不发送
int nineNumber_int_3frame[3] = { 0,0,1 };  // 缓存3帧九宫格数字，如果后两个数相等，则认为九宫格改变
Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

void onChanged(int, void*);

int main()
{
	// 读取 video.cfg 里面的键值对
	bool read = ReadConfig("video.txt", config);
	if (!read) {
		cout << "无法读取 video.cfg" << endl;
		return -1;
	}

	thresh = atoi(config["THRESHOLD"].c_str());
	
	// 初始化串口类
	Serialport Serialport1("/dev/ttyTHS0");
	int fd = Serialport1.open_port("/dev/ttyTHS0");
	if (fd >= 0)
		Serialport1.set_opt(115200, 8, 'N', 1);
 	else
		cout << "open serialport : failed" << endl;
	
	// 初始化摄像头
	VideoCapture cap(0);

	if (!cap.isOpened()) {
		cout << "cannot open video file" << endl;
		return -1;
	}

// 	cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
// 	cap.set(CV_CAP_PROP_FPS, 30);
// 	cap.set(CAP_PROP_FRAME_WIDTH, Width);
// 	cap.set(CAP_PROP_FRAME_HEIGHT, Height);
	
	element0 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

#ifdef DEBUG
	namedWindow("canny");
	createTrackbar("t1", "canny", &t1, 255, onChanged);
	createTrackbar("t2", "canny", &t2, 255, onChanged);
#endif // DEBUG

	/************************************************************************/
	/*                       初始化KNearest数字识别                          */
	/************************************************************************/
	FileStorage fsClassifications("classifications.xml", FileStorage::READ);  // 读取 classifications.xml 分类文件

	if (!fsClassifications.isOpened()) {
		cout << "ERROR, Cannot open file : classifications.xml\n\n";
		//system("pause");
		return 0;
	}

	Mat matClassificationInts;
	fsClassifications["classifications"] >> matClassificationInts;  // 把 classifications.xml 中的 classifications 读取进Mat变量
	fsClassifications.release();  // 关闭文件

	FileStorage fsTrainingImages("images.xml", FileStorage::READ);  // 打开训练图片文件

	if (!fsTrainingImages.isOpened()) {
		cout << "ERROR, Cannot open file : images.xml\n\n";
		//system("pause");
		return 0;
	}

	// 读取训练图片数据（从images.xml中）
	Mat matTrainingImagesAsFlattenedFloats;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages["images"] >> matTrainingImagesAsFlattenedFloats;  // 把 images.xml 中的 images 读取进Mat变量
	fsTrainingImages.release();

	// 训练
	Ptr<ml::KNearest> kNearest(ml::KNearest::create());  // 实例化 kNearest 对象

	// 最终调用train函数，注意到两个参数都是Mat类型（单个Mat），尽管实际上他们都是多张图片或多个数
	kNearest->train(matTrainingImagesAsFlattenedFloats, ml::ROW_SAMPLE, matClassificationInts);

	// 这里计算目标图像的HOG特征（方向梯度直方图特征）代替之前用的灰度特征
	HOGDescriptor *hog = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors;

	while (true)
	{
		cap >> frame;
		if (frame.empty())
			break;

		cvtColor(frame, gray_img, COLOR_BGR2GRAY);
		Canny(gray_img, canny_img, t1, t2);
		dilate(canny_img, canny_img, element0);	// 膨胀

#ifdef DEBUG
		imshow("canny", canny_img);
#endif // DEBUG

		vector<vector<Point> > contours0; // 所有轮廓
		findContours(canny_img, contours0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// 先用面积约束，面积在指定范围内的轮廓的数量不满足要求时，进入下一帧
		vector<vector<Point> > contours1; // 面积在指定范围内的轮廓（九宫格区）
		vector<vector<Point> > contours3; // 面积在指定范围内的轮廓（密码区）
		for (int i = 0; i < contours0.size(); i++) {
			double area = contourArea(contours0[i]);
			if (area > MinArea && area < MaxArea) {		//2000~4500
				contours1.push_back(contours0[i]);
			}
			else if (area > 3000 && area < 3500) {	//5000~7500
				contours3.push_back(contours0[i]);
			}
		}

		if (contours3.size() == 0 || contours1.size() < 9) {
#ifdef DEBUG
			key = waitKey(1);
			if (key == 27)
				break;
#endif // DEBUG
			continue;
		}

		/* 前面findContours寻找所有轮廓（内部、外部），会出现某个宫格找到内外两个轮廓的情况
		 * 这里将满足面积要求的所有轮廓绘制在一个新的图上（这样内部轮廓就会被外部轮廓覆盖）
		 * 然后寻找所有外部轮廓
		 */
		Mat contours_Mat(Height, Width, CV_8UC1, Scalar(0));
		drawContours(contours_Mat, contours1, -1, Scalar(255), -1, LINE_AA);
		erode(contours_Mat, contours_Mat, element0);
		//imshow("contours_Mat", contours_Mat);
		vector<vector<Point> > contours2; // 面积在指定范围内的轮廓（九宫格区_）
		findContours(contours_Mat, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		Mat contours_Mat1(Height, Width, CV_8UC1, Scalar(0));
		drawContours(contours_Mat1, contours3, -1, Scalar(255), -1, LINE_AA);
		//imshow("contours_Mat1", contours_Mat1);
		vector<vector<Point> > contours4; // 面积在指定范围内的轮廓（密码区_）
		findContours(contours_Mat1, contours4, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		if (contours4.size() == 0 || contours2.size() < 9) {
#ifdef DEBUG
			key = waitKey(1);
			if (key == 27)
				break;
#endif // DEBUG
			continue;
		}

		/************************************************************************/
		/*                            检测九宫格区                               */
		/************************************************************************/

		/* 此时，contours2.size() >= 9，contours4.size() >=1
	     * 再对宫格使用长宽比约束和角度约束
		 */
		
		// 对面积在指定范围内的轮廓取最小包围矩形（九宫格区）
		vector<RotatedRect> contours_rotatedRect;
		for (int i = 0; i < contours2.size(); i++) {
			contours_rotatedRect.push_back(minAreaRect(contours2[i]));
		}

		// 如果多于9个轮廓，需要使用其他约束排除
		if (contours2.size() > 9) {
			//cout << "more than 9" << endl;
			vector<RotatedRect> contours_rotatedRect_tmp;
			for (int i = 0; i < contours_rotatedRect.size(); i++) {
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

				if (tmp_float > 1.40 && tmp_float < 1.90) {
					b2 = true;
				}

				if (tmp_area > MinArea && tmp_area < (MaxArea+200)) {
					b3 = true;
				}
				
				if (b1 && b2 && b3)
					contours_rotatedRect_tmp.push_back(contours_rotatedRect[i]);
			}
			contours_rotatedRect.clear();
			contours_rotatedRect = contours_rotatedRect_tmp;
		}

		/* 经过约束去除掉一些轮廓（旋转矩形）
		 * 如果剩下的轮廓数量等于9，则认为是九宫格，并提取数字的Mat
		 * 否则进入下一帧
		 */
		vector<Rect> nineRect(9);		// 修正过的九宫格（数字）的外接矩形
		vector<Mat> nineRect_mat(9);	// 经过阈值分割和膨胀处理的九宫格（数字）的Mat
		if (contours_rotatedRect.size() == 9) {
			for (int i = 0; i < 9; i++) {
				Rect objectBoundary = contours_rotatedRect[i].boundingRect();
				//修正，objectBoundary向右下角移动（向右17，向下10），再缩小（右侧38，下侧19）
				objectBoundary += Point(12, 5);
				objectBoundary -= Size(24, 10);
				nineRect[i] = objectBoundary;
			}

			RectSort(nineRect);		// 对九个Rect进行排序

			for (int i = 0; i < 9; i++) {
				threshold(gray_img(nineRect[i]), nineRect_mat[i], thresh, 255, THRESH_BINARY_INV);
				dilate(nineRect_mat[i], nineRect_mat[i], element1);		// 膨胀
				rectangle(frame, nineRect[i], Scalar(204, 122, 0), 2, LINE_AA);
				char distance[10];
				snprintf(distance, 10, "%d", i + 1);
				putText(frame, distance, Point(nineRect[i].x, nineRect[i].y), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.6, Scalar(0, 0, 255), 2, 8);
			}
		}
		else {
			imshow("bgr", frame);
			key = waitKey(1);
			if (key == 27)
				break;
			continue;
		}

		/* 此时已经提取到9个数字的Rect和Mat，并按照宫格的位置排好顺序
		 * 下面对依次对数字进行识别
		 */
		for (int i = 0; i < 9; i++) {
			Mat number_mat = nineRect_mat[i].clone();
			vector<vector<Point> > number_contour;
			findContours(number_mat, number_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			for (int j = 0; j < number_contour.size(); j++) {
				// 由于包含有数字的Mat里可能出现其他细线或小块，这里用面积将它们排除
				if (contourArea(number_contour[j]) > 80) {
					Rect rect_tmp = boundingRect(number_contour[j]);
					Mat numImage = nineRect_mat[i](rect_tmp);
					resize(numImage, numImage, Size(40, 40));
					hog->compute(numImage, descriptors);
					Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());

					Mat matCurrentChar(0, 0, CV_32F);  // findNearest的结果保存在这里

					// 最终调用 findNearest 函数
					kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar);

					nineNumber[i] = (int)matCurrentChar.at<float>(0, 0);
				}
			}
		}

// 		if (first && key == int('0')) {
// 			for (int i = 0; i < 9; i++) {
// 				imwrite(to_string(i) + ".png", nineRect_mat[i]);
// 			}
// 			first = false;
// 		}
		//cout << endl;

		/************************************************************************/
		/*                            检测密码区                                 */
		/************************************************************************/
		
		// 密码区的旋转矩形
		vector<RotatedRect> contours_rotatedRect1;
		for (int i = 0; i < contours3.size(); i++) {
			contours_rotatedRect1.push_back(minAreaRect(contours3[i]));
		}

		if (contours3.size() > 1) {
			vector<RotatedRect> contours_rotatedRect1_tmp;
			for (int i = 0; i < contours_rotatedRect1.size(); i++) {
				bool b1 = false, b2 = false;

				float tmp_float = contours_rotatedRect1[i].size.width / contours_rotatedRect1[i].size.height;
				if (tmp_float < 1) {
					tmp_float = 1.0 / tmp_float;
					float tmp_float = contours_rotatedRect1[i].size.height;
					contours_rotatedRect1[i].size.height = contours_rotatedRect1[i].size.width;
					contours_rotatedRect1[i].size.width = tmp_float;
					contours_rotatedRect1[i].angle = contours_rotatedRect1[i].angle + 90;
				}

				if (contours_rotatedRect1[i].angle > -5 && contours_rotatedRect1[i].angle < 5) {
					b1 = true;
				}

				if (tmp_float > 4.0 && tmp_float < 4.5) {
					b2 = true;
				}
				if (b1 && b2)
					contours_rotatedRect1_tmp.push_back(contours_rotatedRect1[i]);
			}
			contours_rotatedRect1.clear();
			contours_rotatedRect1 = contours_rotatedRect1_tmp;
		}
		else if (contours2.size() < 1) {
			//cout << "less than 1" << endl;
			imshow("bgr", frame);
			key = waitKey(1);
			if (key == 27)
				break;
			continue;
		}

		if (contours_rotatedRect1.size() == 1) {
			vector<Point2f> p(4);
			contours_rotatedRect1[0].points(p.data());

			sortPoints(p);
			for (int i = 0; i < 4; i++) {
				line(frame, p[i], p[(i + 1) % 4], Scalar(255, 0, 0), 2, LINE_AA);
			}

			Point2f srcPoints[4];
			Point2f dstPoints[4];

			srcPoints[0] = p[0] + Point2f(10, 3);
			srcPoints[1] = p[1] + Point2f(-10, 3);
			srcPoints[2] = p[2] + Point2f(-10, -2);
			srcPoints[3] = p[3] + Point2f(10, -2);

			dstPoints[0] = Point2f(0, 0);
			dstPoints[1] = Point2f(160, 0);
			dstPoints[2] = Point2f(160, 40);
			dstPoints[3] = Point2f(0, 40);

			Mat warpMat(2, 4, CV_32FC1);
			warpMat = getPerspectiveTransform(srcPoints, dstPoints);

			// 透视变换，将密码区变换成160*40的Mat
			Mat dstImage(40, 160, CV_8UC1, Scalar(0));
			warpPerspective(gray_img, dstImage, warpMat, dstImage.size());

			Mat dstImage_bin;
			threshold(dstImage, dstImage_bin, thresh, 255, THRESH_BINARY_INV);
			dilate(dstImage_bin, dstImage_bin, element0);

#ifdef DEBUG
			Mat dst_img(Height, Width, CV_8UC1, Scalar(0));
			for (int i = 0; i < 9; i++) {
				nineRect_mat[i].copyTo(dst_img(nineRect[i]));
			}
			dstImage_bin.copyTo(dst_img(Rect(240, 0, 160, 40)));
			imshow("dst", dst_img);
#endif // DEBUG

			Mat dstImage_bin_ = dstImage_bin.clone();
			vector<vector<Point> > number_contours;
			findContours(dstImage_bin_, number_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			sortContours(number_contours);	// 将包含数字的5个轮廓按照从左到右的顺序排序
			for (int i = 0; i < number_contours.size(); i++) {
				if (contourArea(number_contours[i]) < 30)
					continue;
				Rect boundRect = boundingRect(number_contours[i]);
				Mat numImage = dstImage_bin(boundRect);
				resize(numImage, numImage, Size(40, 40));
				hog->compute(numImage, descriptors);
				Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());

				Mat matCurrentChar(0, 0, CV_32F);  // findNearest的结果保存在这里

				// 最终调用 findNearest 函数
				kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar);

				password[i] = (int)matCurrentChar.at<float>(0, 0);
			}
		}

#ifdef DEBUG
		// 输出密码区和九宫格区的数字
		for (int i = 0; i < 5; i++)
			cout << password[i];
		cout << endl;
		for (int i = 0; i < 9; i++)
			cout << nineNumber[i];
		cout << endl;
#endif // DEBUG

		// 判断密码区是否改变，如果前两个数改变则认为变了
		if (password_int != (password[0] * 10 + password[1]))
			password_changed = true;
		else
			password_changed = false;

		// 判断九宫格区是否改变，如果前三个数改变则认为变了
		nineNumber_int_3frame[0] = nineNumber_int_3frame[1];
		nineNumber_int_3frame[1] = nineNumber_int_3frame[2];
		nineNumber_int_3frame[2] = nineNumber[0] * 100 + nineNumber[1] * 10 + nineNumber[2];
		if (nineNumber_int_3frame[0] != nineNumber_int_3frame[1] && nineNumber_int_3frame[2] == nineNumber_int_3frame[1])
			nineNumber_changed = true;
		else
			nineNumber_changed = false;

		if (password_changed) {
			count_ = 0;
		} else {
			if (count_ < 4 && nineNumber_changed)
				count_++;
		}
		
		for (int i = 0; i < 9; i++) {
			if (password[count_] == nineNumber[i]) {
				targetNum[0] = targetNum[1];
				targetNum[1] = targetNum[2];
				targetNum[2] = i;

				if (targetNum[0] == targetNum[1] && targetNum[1] == targetNum[2]) {
					cout << "targetNum : " << i + 1;
					Serialport1.usart3_send(static_cast<uint8_t>(i + 1));
					rectangle(frame, nineRect[i], Scalar(0, 0, 255), 3, LINE_AA);
				}
				break;
			}
		}
		if (password_changed)
			count_ = -1;

		password_int = password[0] * 10 + password[1];
		nineNumber_int = nineNumber[0] * 100 + nineNumber[1] * 10 + nineNumber[2];

		cout << "\n---------" << endl;
		imshow("bgr", frame);

		key = waitKey(20);
		if (key == 27)
			break;
		else if (key == int('0'))
			waitKey(0);
	}
}

void onChanged(int, void*) {
	Canny(gray_img, canny_img, t1, t2);
	dilate(canny_img, canny_img, element0);	// 膨胀
	imshow("canny", canny_img);
}
