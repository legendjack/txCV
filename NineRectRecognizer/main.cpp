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
int t1 = 200, t2 = 250;			// canny��ֵ
int key;						// waitKey()����ֵ
int password[5];				// ��������5����
int password_int = 0;			// ������ǰ��������ɵ���λ��������ı�����Ϊ����仯
bool password_changed = true;	// �������Ƿ�ı�	
bool nineNumber_changed = true; // �Ź������Ƿ�ı�
int nineNumber[9];				// �Ź�������9����
int nineNumber_int = 0;			// �Ź����һ�е���������ɵ���λ��������仯����Ϊ�Ź������ı�
int count_ = 0;					// �����ã���ǰĿ������������count_����
int targetNum[3] = { 0,0,1 };	// ����3֡�Ľ�Ҫ���͵����ݣ������������ȣ����ͣ�������Ϊ��һ֡��죬������
int nineNumber_int_3frame[3] = { 0,0,1 };  // ����3֡�Ź������֣��������������ȣ�����Ϊ�Ź���ı�
Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));

void onChanged(int, void*);

int main()
{
	// ��ȡ video.cfg ����ļ�ֵ��
	bool read = ReadConfig("video.txt", config);
	if (!read) {
		cout << "�޷���ȡ video.cfg" << endl;
		return -1;
	}

	thresh = atoi(config["THRESHOLD"].c_str());
	
	// ��ʼ��������
	Serialport Serialport1("/dev/ttyTHS0");
	int fd = Serialport1.open_port("/dev/ttyTHS0");
	if (fd >= 0)
		Serialport1.set_opt(115200, 8, 'N', 1);
 	else
		cout << "open serialport : failed" << endl;
	
	// ��ʼ������ͷ
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
	/*                       ��ʼ��KNearest����ʶ��                          */
	/************************************************************************/
	FileStorage fsClassifications("classifications.xml", FileStorage::READ);  // ��ȡ classifications.xml �����ļ�

	if (!fsClassifications.isOpened()) {
		cout << "ERROR, Cannot open file : classifications.xml\n\n";
		//system("pause");
		return 0;
	}

	Mat matClassificationInts;
	fsClassifications["classifications"] >> matClassificationInts;  // �� classifications.xml �е� classifications ��ȡ��Mat����
	fsClassifications.release();  // �ر��ļ�

	FileStorage fsTrainingImages("images.xml", FileStorage::READ);  // ��ѵ��ͼƬ�ļ�

	if (!fsTrainingImages.isOpened()) {
		cout << "ERROR, Cannot open file : images.xml\n\n";
		//system("pause");
		return 0;
	}

	// ��ȡѵ��ͼƬ���ݣ���images.xml�У�
	Mat matTrainingImagesAsFlattenedFloats;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages["images"] >> matTrainingImagesAsFlattenedFloats;  // �� images.xml �е� images ��ȡ��Mat����
	fsTrainingImages.release();

	// ѵ��
	Ptr<ml::KNearest> kNearest(ml::KNearest::create());  // ʵ���� kNearest ����

	// ���յ���train������ע�⵽������������Mat���ͣ�����Mat��������ʵ�������Ƕ��Ƕ���ͼƬ������
	kNearest->train(matTrainingImagesAsFlattenedFloats, ml::ROW_SAMPLE, matClassificationInts);

	// �������Ŀ��ͼ���HOG�����������ݶ�ֱ��ͼ����������֮ǰ�õĻҶ�����
	HOGDescriptor *hog = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors;

	while (true)
	{
		cap >> frame;
		if (frame.empty())
			break;

		cvtColor(frame, gray_img, COLOR_BGR2GRAY);
		Canny(gray_img, canny_img, t1, t2);
		dilate(canny_img, canny_img, element0);	// ����

#ifdef DEBUG
		imshow("canny", canny_img);
#endif // DEBUG

		vector<vector<Point> > contours0; // ��������
		findContours(canny_img, contours0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// �������Լ���������ָ����Χ�ڵ�����������������Ҫ��ʱ��������һ֡
		vector<vector<Point> > contours1; // �����ָ����Χ�ڵ��������Ź�������
		vector<vector<Point> > contours3; // �����ָ����Χ�ڵ���������������
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

		/* ǰ��findContoursѰ�������������ڲ����ⲿ���������ĳ�������ҵ������������������
		 * ���ｫ�������Ҫ�����������������һ���µ�ͼ�ϣ������ڲ������ͻᱻ�ⲿ�������ǣ�
		 * Ȼ��Ѱ�������ⲿ����
		 */
		Mat contours_Mat(Height, Width, CV_8UC1, Scalar(0));
		drawContours(contours_Mat, contours1, -1, Scalar(255), -1, LINE_AA);
		erode(contours_Mat, contours_Mat, element0);
		//imshow("contours_Mat", contours_Mat);
		vector<vector<Point> > contours2; // �����ָ����Χ�ڵ��������Ź�����_��
		findContours(contours_Mat, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		Mat contours_Mat1(Height, Width, CV_8UC1, Scalar(0));
		drawContours(contours_Mat1, contours3, -1, Scalar(255), -1, LINE_AA);
		//imshow("contours_Mat1", contours_Mat1);
		vector<vector<Point> > contours4; // �����ָ����Χ�ڵ�������������_��
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
		/*                            ���Ź�����                               */
		/************************************************************************/

		/* ��ʱ��contours2.size() >= 9��contours4.size() >=1
	     * �ٶԹ���ʹ�ó����Լ���ͽǶ�Լ��
		 */
		
		// �������ָ����Χ�ڵ�����ȡ��С��Χ���Σ��Ź�������
		vector<RotatedRect> contours_rotatedRect;
		for (int i = 0; i < contours2.size(); i++) {
			contours_rotatedRect.push_back(minAreaRect(contours2[i]));
		}

		// �������9����������Ҫʹ������Լ���ų�
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

		/* ����Լ��ȥ����һЩ��������ת���Σ�
		 * ���ʣ�µ�������������9������Ϊ�ǾŹ��񣬲���ȡ���ֵ�Mat
		 * ���������һ֡
		 */
		vector<Rect> nineRect(9);		// �������ľŹ������֣�����Ӿ���
		vector<Mat> nineRect_mat(9);	// ������ֵ�ָ�����ʹ���ľŹ������֣���Mat
		if (contours_rotatedRect.size() == 9) {
			for (int i = 0; i < 9; i++) {
				Rect objectBoundary = contours_rotatedRect[i].boundingRect();
				//������objectBoundary�����½��ƶ�������17������10��������С���Ҳ�38���²�19��
				objectBoundary += Point(12, 5);
				objectBoundary -= Size(24, 10);
				nineRect[i] = objectBoundary;
			}

			RectSort(nineRect);		// �ԾŸ�Rect��������

			for (int i = 0; i < 9; i++) {
				threshold(gray_img(nineRect[i]), nineRect_mat[i], thresh, 255, THRESH_BINARY_INV);
				dilate(nineRect_mat[i], nineRect_mat[i], element1);		// ����
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

		/* ��ʱ�Ѿ���ȡ��9�����ֵ�Rect��Mat�������չ����λ���ź�˳��
		 * ��������ζ����ֽ���ʶ��
		 */
		for (int i = 0; i < 9; i++) {
			Mat number_mat = nineRect_mat[i].clone();
			vector<vector<Point> > number_contour;
			findContours(number_mat, number_contour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
			for (int j = 0; j < number_contour.size(); j++) {
				// ���ڰ��������ֵ�Mat����ܳ�������ϸ�߻�С�飬����������������ų�
				if (contourArea(number_contour[j]) > 80) {
					Rect rect_tmp = boundingRect(number_contour[j]);
					Mat numImage = nineRect_mat[i](rect_tmp);
					resize(numImage, numImage, Size(40, 40));
					hog->compute(numImage, descriptors);
					Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());

					Mat matCurrentChar(0, 0, CV_32F);  // findNearest�Ľ������������

					// ���յ��� findNearest ����
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
		/*                            ���������                                 */
		/************************************************************************/
		
		// ����������ת����
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

			// ͸�ӱ任�����������任��160*40��Mat
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
			sortContours(number_contours);	// ���������ֵ�5���������մ����ҵ�˳������
			for (int i = 0; i < number_contours.size(); i++) {
				if (contourArea(number_contours[i]) < 30)
					continue;
				Rect boundRect = boundingRect(number_contours[i]);
				Mat numImage = dstImage_bin(boundRect);
				resize(numImage, numImage, Size(40, 40));
				hog->compute(numImage, descriptors);
				Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());

				Mat matCurrentChar(0, 0, CV_32F);  // findNearest�Ľ������������

				// ���յ��� findNearest ����
				kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar);

				password[i] = (int)matCurrentChar.at<float>(0, 0);
			}
		}

#ifdef DEBUG
		// ����������;Ź�����������
		for (int i = 0; i < 5; i++)
			cout << password[i];
		cout << endl;
		for (int i = 0; i < 9; i++)
			cout << nineNumber[i];
		cout << endl;
#endif // DEBUG

		// �ж��������Ƿ�ı䣬���ǰ�������ı�����Ϊ����
		if (password_int != (password[0] * 10 + password[1]))
			password_changed = true;
		else
			password_changed = false;

		// �жϾŹ������Ƿ�ı䣬���ǰ�������ı�����Ϊ����
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
	dilate(canny_img, canny_img, element0);	// ����
	imshow("canny", canny_img);
}
