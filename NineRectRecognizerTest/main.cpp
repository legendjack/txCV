#include "functions.h"

#define DEBUG

const int Width = 800;		// ��Ƶ��
const int Height = 600;		// ��Ƶ��
const int MaxArea = 8000;	// ����������������
const int MinArea = 4000;	// ������������С���

VideoCapture cap;
Mat frame, gray_img, canny_img;
Mat element0, element1;

int t1 = 200, t2 = 250;		// canny��ֵ
int nineNumber[9];			// �Ź������� 9 ������
int password[5];			// ������������ܣ��� 5 ������

Rect passwordRect(0, 0, 200, 60);	// �Ź������� Rect
Mat pw_gray, pw_bin;				// ������������ܣ��ĻҶ�ͼ�Ͷ�ֵͼ
bool foundNixieTubeArea = false;	// �Ƿ����������

int main(int argc, char** argv)
{
	cap.open("output3.avi");
	cap.set(CAP_PROP_POS_FRAMES, 2 * 30);

	element0 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));

	/************************************************************************/
	/*                     ��ʼ��KNearest����ʶ����д�壩                   */
	/************************************************************************/
	FileStorage fsClassifications("classifications.xml", FileStorage::READ);  // ��ȡ classifications.xml �����ļ�

	if (!fsClassifications.isOpened()) {
		cout << "ERROR, �޷���classifications.xml\n\n";
		//system("pause");
		return 0;
	}

	Mat matClassificationInts;
	fsClassifications["classifications"] >> matClassificationInts;  // �� classifications.xml �е� classifications ��ȡ��Mat����
	fsClassifications.release();  // �ر��ļ�

	FileStorage fsTrainingImages("images.xml", FileStorage::READ);  // ��ѵ��ͼƬ�ļ�

	if (!fsTrainingImages.isOpened()) {
		cout << "ERROR, �޷���images.xml\n\n";
		//system("pause");
		return 0;
	}

	// ��ȡѵ��ͼƬ���ݣ���images.xml�У�
	Mat matTrainingImagesAsFlattenedFloats;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages["images"] >> matTrainingImagesAsFlattenedFloats;  // �� images.xml �е� images ��ȡ��Mat����
	fsTrainingImages.release();

	// ѵ��
	Ptr<ml::KNearest> kNearest(ml::KNearest::create());  // ʵ���� kNearest ����

	// ���յ���train����
	kNearest->train(matTrainingImagesAsFlattenedFloats, ml::ROW_SAMPLE, matClassificationInts);

	// ����Ŀ��ͼ���HOG�����������ݶ�ֱ��ͼ����������֮ǰ�õĻҶ�����
	HOGDescriptor *hog = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors;

	/************************************************************************/
	/*                  ��ʼ��KNearest����ʶ������ܣ�                      */
	/************************************************************************/
	FileStorage fsClassifications1("classifications1.xml", FileStorage::READ);
	if (!fsClassifications1.isOpened()) {
		cout << "ERROR, �޷���classifications1.xml\n\n";
		return 0;
	}
	Mat matClassificationInts1;
	fsClassifications1["classifications"] >> matClassificationInts1;
	fsClassifications1.release();
	FileStorage fsTrainingImages1("images1.xml", FileStorage::READ);
	if (!fsTrainingImages1.isOpened()) {
		cout << "ERROR, �޷���images.xml\n\n";
		return 0;
	}
	Mat matTrainingImagesAsFlattenedFloats1;  // we will read multiple images into this single image variable as though it is a vector
	fsTrainingImages1["images"] >> matTrainingImagesAsFlattenedFloats1;  // �� images.xml �е� images ��ȡ��Mat����
	fsTrainingImages1.release();
	Ptr<ml::KNearest> kNearest1(ml::KNearest::create());
	kNearest1->train(matTrainingImagesAsFlattenedFloats1, ml::ROW_SAMPLE, matClassificationInts1);
	HOGDescriptor *hog1 = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors1;

	/************************************************************************/
	/*                         ��ʼ���ÿһ֡ͼ��                             */
	/************************************************************************/
	while (cap.isOpened()) {
		cap >> frame;
		//imwrite("frame.jpg", frame);

		if (frame.empty())
			break;
		imshow("frame", frame);

		cvtColor(frame, gray_img, COLOR_BGR2GRAY);
		Canny(gray_img, canny_img, t1, t2);
		dilate(canny_img, canny_img, element0);	// ����
		imshow("canny", canny_img);

		vector<vector<Point> > contours0;		// ��������
		findContours(canny_img, contours0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// �������Լ���������ָ����Χ�ڵ�����������������Ҫ��ʱ��������һ֡
		vector<vector<Point> > contours1;		// �����ָ����Χ�ڵ��������Ź�������
		vector<double> areas;
		for (size_t i = 0; i < contours0.size(); i++) {
			double area = contourArea(contours0[i]);
			areas.push_back(area);
			if (area > MinArea && area < MaxArea) {
				contours1.push_back(contours0[i]);
			}
		}

	 	if (contours1.size() < 9) {
			cout << "�����ָ����Χ�ڵ�������������" << endl;
			for (size_t i = 0; i < areas.size(); i++)
				if (areas[i] > 1000)
					cout << areas[i] << " ";
			cout << endl;
			continue;
		}

		// ����õ���������������͹ȱ�ݵģ�������ֵıʻ����쵽�������ı�Ե��
		// ������ҪѰ��͹�������ư���͹ȱ�ݵĽϹ淶�ľ���
		Mat contours_Mat(Height, Width, CV_8UC1, Scalar(0));
		for (size_t i = 0; i < contours1.size(); i++) {
			vector<int> hull;						// �ҵ���͹������ʵ�������������������
			convexHull(contours1[i], hull, true);	// Ѱ�ҵ㼯����������͹��
			int hullcount = (int)hull.size();
			Point point0 = contours1[i][hull[hullcount - 1]];
			for (int j = 0; j < hullcount; j++) {
				Point point = contours1[i][hull[j]];
				line(contours_Mat, point0, point, Scalar(255), 1, LINE_AA);
				point0 = point;
			}
		}
		imshow("contours_Mat", contours_Mat);
		vector<vector<Point> > contours2;				// �����ָ����Χ�ڵ��������Ź�����_��
		findContours(contours_Mat, contours2, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		
		// �������ָ����Χ�ڵ�����ȡ��С��Χ���Σ��Ź�������
		vector<RotatedRect> contours_rotatedRect;
		for (int i = 0; i < contours2.size(); i++) {
			// �����ж��� contours2[i].size() �ķ�Χ����� contours2[i] �ǽϹ淶�ľ��Σ��� contours2[i].size() < 100
			if (contours2[i].size() < 100)
				contours_rotatedRect.push_back(minAreaRect(contours2[i]));
		}

		// �����ת���ε���������9������һЩԼ�����Ƕȣ������
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

		vector<Mat> nineRect_mat(9);	// ������ֵ�ָ�����ʹ���ľŹ������֣���Mat
		if (contours_rotatedRect.size() == 9) {
			sortRotatedRect(contours_rotatedRect);	// ��9����ת���ΰ���˳������
			// ����ÿ����ת���Σ��ҵ����ĸ����㣬ʹ�÷���任����任Ϊ������
			for (int i = 0; i < 9; i++) {
				vector<Point2f> p(4);
				contours_rotatedRect[i].points(p.data());
				sortPoints(p);

				Point2f srcPoints[4];
				Point2f dstPoints[4];

				srcPoints[0] = p[0] + Point2f(10, 8);
				srcPoints[1] = p[1] + Point2f(-10, 8);
				srcPoints[2] = p[2] + Point2f(-10, -4);
				srcPoints[3] = p[3] + Point2f(10, -4);

				if (i == 0) {
					passwordRect.x = p[1].x - 20;
					passwordRect.y = p[1].y - 80;
				}
				else if (i == 2) {
					passwordRect.width = p[0].x + 15 - passwordRect.x;
					passwordRect.height = p[0].y - 20 - passwordRect.y;
				}

				if (passwordRect.x < 0 || (passwordRect.x + passwordRect.width) > Width ||
					passwordRect.y < 0 || (passwordRect.y + passwordRect.height) > Height)
					foundNixieTubeArea = false;
				else
					foundNixieTubeArea = true;

				for (int j = 0; j < 4; j++)
					line(frame, srcPoints[j], srcPoints[(j + 1) % 4], Scalar(204, 122, 0), 2, LINE_AA);
				//putText(frame, to_string(i + 1), Point(contours_rotatedRect[i].center.x, contours_rotatedRect[i].center.y) - Point(20, 5), FONT_HERSHEY_SCRIPT_SIMPLEX, 0.6, Scalar(66, 206, 255), 2, 8);

				dstPoints[0] = Point2f(0, 0);
				dstPoints[1] = Point2f(40, 0);
				dstPoints[2] = Point2f(40, 40);
				dstPoints[3] = Point2f(0, 40);

				Mat warpMat(2, 4, CV_32FC1);
				warpMat = getPerspectiveTransform(srcPoints, dstPoints);

				// ͸�ӱ任�����������任��40*40��Mat
				Mat dstImage(40, 40, CV_8UC1, Scalar(0));
				warpPerspective(gray_img, dstImage, warpMat, dstImage.size());

				threshold(dstImage, nineRect_mat[i], 0, 255, THRESH_OTSU);
				threshold(nineRect_mat[i], nineRect_mat[i], 100, 255, THRESH_BINARY_INV);
				dilate(nineRect_mat[i], nineRect_mat[i], element0);		// ����
				deskew(nineRect_mat[i]);
				blur(nineRect_mat[i], nineRect_mat[i], Size(3, 3));
			}
		}
		else {
			continue;
		}

		Mat nineNumberMat(120, 120, CV_8UC1);
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				nineRect_mat[j + i * 3].copyTo(nineNumberMat(Rect(j * 40, i * 40, 40, 40)));
		imshow("nineNumberMat", nineNumberMat);

		if (!foundNixieTubeArea)
			continue;

		/************************************************************************/
		/*                     ���������������ܣ�                               */
		/************************************************************************/
		pw_gray = gray_img(passwordRect);
		threshold(pw_gray, pw_bin, 200, 255, THRESH_BINARY);
		erode(pw_bin, pw_bin, element0);
		dilate(pw_bin, pw_bin, element1);
		connectClosedPoint(pw_bin);

		Mat pw_bin_ = pw_bin.clone();
		vector<vector<Point> > ninxiTubeAreaContour;
		findContours(pw_bin_, ninxiTubeAreaContour, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		vector<Rect> ninxiTubeNumbRect;
		for (size_t i = 0; i < ninxiTubeAreaContour.size(); i++) {
			Rect tmpRect = boundingRect(ninxiTubeAreaContour[i]);
			int tmpHeight = MAX(tmpRect.width, tmpRect.height);
			if (tmpHeight > 20)
				ninxiTubeNumbRect.push_back(tmpRect);
		}

		if (ninxiTubeNumbRect.size() == 5) {
			sortRect(ninxiTubeNumbRect);
		}
		else {
			waitKey(1);
			cout << "�����ʶ�����ninxiTubeNumbRect.size() = " << ninxiTubeNumbRect.size() << endl;
			continue;
		}
		
		for (int i = 0; i < 5; i++) {
			Mat matROI = pw_bin(ninxiTubeNumbRect[i]);
			resize(matROI, matROI, Size(40, 40));
			hog1->compute(matROI, descriptors1);
			Mat matROIFlattenedFloat(1, (int)(descriptors1.size()), CV_32FC1, descriptors1.data());
			Mat matCurrentChar(0, 0, CV_32F);
			kNearest1->findNearest(matROIFlattenedFloat, 1, matCurrentChar);
			password[i] = (int)matCurrentChar.at<float>(0, 0);
		}

		for (int i = 0; i < 5; i++)
			cout << password[i];
		cout << "-->";

		imshow("frame", frame);
		imshow("password", pw_bin);

		double time0 = static_cast<double>(getTickCount());
		
		for (int i = 0; i < 9; i++) {
			hog->compute(nineRect_mat[i], descriptors);
			Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());
			Mat matCurrentChar(0, 0, CV_32F);  // findNearest�Ľ������������
			kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar);
			nineNumber[i] = (int)matCurrentChar.at<float>(0, 0);
		}

		time0 = ((double)getTickCount() - time0) / getTickFrequency();
		cout << "time : " << time0 * 1000 << "ms" << endl;
		
		for (int i = 0; i < 9; i++)
			cout << nineNumber[i];
		cout << endl;

		int key = waitKey(1);
		if (key == 32)
			waitKey(0);
		else if (key == 27)
			break;
		
		
	}

	return 0;
}
/*
double time0 = static_cast<double>(getTickCount());
time0 = ((double)getTickCount() - time0) / getTickFrequency();
cout << "time : " << time0 * 1000 << "ms" << endl;
*/
