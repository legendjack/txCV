#include "functions.h"
#include "serialsom.h"

#define DEBUG

const int Width = 800;		// ��Ƶ��
const int Height = 600;		// ��Ƶ��

int MaxArea = 8000;			// ÿ������������������
int MinArea = 4000;			// ÿ��������������С���
int minGrayValue;			// �����ڵ���ͻҶ�ֵ�����С�ڸ�ֵ����Ϊ������û�����ݣ����֣�

VideoCapture cap;
Mat frame, gray_img, canny_img;
Mat element0, element1, element2;

int t1 = 200, t2 = 250;		// canny��ֵ
int password[5];			// ������������ܣ��� 5 ������
int nineNumber[9];			// �Ź������� 9 ������
float neighborDistance[9];	// kNN ʶ��ÿ������������ھӵľ��룬ֵԽС˵���Ǹ�ֵ�Ŀ�����Խ��
int errorCount;				// ʶ��������ֵĸ���

Rect passwordRect(0, 0, 200, 60);	// �Ź������� Rect
Mat pw_gray, pw_bin;				// ������������ܣ��ĻҶ�ͼ�Ͷ�ֵͼ
bool foundNixieTubeArea = false;	// �Ƿ����������
bool isEmpty = false;				// �������Ƿ�������

int nineRectNumber = 0;				// �ɾŹ���ǰ��λ������ɵ���λ��������仯���ʾ�Ź������ı�
int targetRect = 1;					// ��ǰĿ�깬��1~9

int main(int argc, char** argv)
{
	// ��ʼ������
	Serialport Serialport1("/dev/ttyTHS0");
	int fd = Serialport1.open_port("/dev/ttyTHS0");
	if (fd >= 0)
		Serialport1.set_opt(115200, 8, 'N', 1);
 	else
		cout << "open serialport : failed" << endl;
	
	// ��ȡ�����ļ�
	FileStorage fs("config.xml", FileStorage::READ);

	string filename;
	fs["filename"] >> filename;
	fs["minGrayValue"] >> minGrayValue;
	fs["t1"] >> t1;
	fs["t2"] >> t2;

	fs.release();
	
	// ������ͷ
 	cap.open(0);
 	cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap.set(CAP_PROP_FRAME_WIDTH, Width);
 	cap.set(CAP_PROP_FRAME_HEIGHT, Height);
//	cap.open(filename);

	element0 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
	element2 = getStructuringElement(MORPH_ELLIPSE, Size(2, 2));
	
	/************************************************************************/
	/*                     ��ʼ��KNearest����ʶ����д�壩                   */
	/************************************************************************/
	FileStorage fsClassifications("classifications.xml", FileStorage::READ);  // ��ȡ classifications.xml �����ļ�

	if (!fsClassifications.isOpened()) {
		cout << "ERROR, cannot open classifications.xml\n\n";
		//system("pause");
		return 0;
	}

	Mat matClassificationInts;
	fsClassifications["classifications"] >> matClassificationInts;  // �� classifications.xml �е� classifications ��ȡ��Mat����
	fsClassifications.release();  // �ر��ļ�

	FileStorage fsTrainingImages("images.xml", FileStorage::READ);  // ��ѵ��ͼƬ�ļ�

	if (!fsTrainingImages.isOpened()) {
		cout << "ERROR, cannot open images.xml\n\n";
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
	fsTrainingImages1["images"] >> matTrainingImagesAsFlattenedFloats1;  // �� images.xml �е� images ��ȡ��Mat����
	fsTrainingImages1.release();
	Ptr<ml::KNearest> kNearest1(ml::KNearest::create());
	kNearest1->train(matTrainingImagesAsFlattenedFloats1, ml::ROW_SAMPLE, matClassificationInts1);
	HOGDescriptor *hog1 = new HOGDescriptor(Size(40, 40), Size(16, 16), Size(8, 8), Size(8, 8), 9);
	vector<float> descriptors1;

	/************************************************************************/
	/*                         ��ʼ���ÿһ֡ͼ��                             */
	/************************************************************************/
	while (true) {
		cap >> frame;

		if (frame.empty())
			break;	// continue

		cvtColor(frame, gray_img, COLOR_BGR2GRAY);
		Canny(gray_img, canny_img, t1, t2);
		dilate(canny_img, canny_img, element0);	// ����
		
#ifdef DEBUG
		imshow("frame", frame);
		imshow("canny", canny_img);
#endif

		// Ѱ����������
		vector<vector<Point> > contours0;		// ��������
		findContours(canny_img, contours0, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

		// �������Լ���������ָ����Χ�ڵ�����������������Ҫ��ʱ��������һ֡
		vector<vector<Point> > contours1;		// �����ָ����Χ�ڵ��������Ź�������
		for (size_t i = 0; i < contours0.size(); i++) {
			double area = contourArea(contours0[i]);
			if (area > MinArea && area < MaxArea) {
				contours1.push_back(contours0[i]);
			}
		}

	 	if (contours1.size() < 9) {
			cout << "contours in specified range are not enough" << endl;
			continue;
		}

		// ����õ�������������͹ȱ�ݣ�������ֵıʻ����쵽�������ı�Ե��
		// ������ҪѰ��͹�������ư���͹ȱ�ݵĽϹ淶�ľ���
		vector<vector<Point> > contours2;			// �����ָ����Χ�ڵ��������Ź�����_��
		for (size_t i = 0; i < contours1.size(); i++) {
			vector<int> hull;						// �ҵ���͹������ʵ�������������������
			convexHull(contours1[i], hull, true);	// Ѱ�ҵ㼯����������͹��

			vector<Point> tempContour;				// ��͹�������㼯�������������ڼ������
			for (size_t j = 0; j < hull.size(); j++)
				tempContour.push_back(contours1[i][hull[j]]);
			double tempArea = contourArea(tempContour);
			if (tempArea < 4500 || tempArea > 7500)	// ���Լ��
				continue;

			vector<Point> tempContour2;
			approxPolyDP(tempContour, tempContour2, 30, true);
			if (tempContour2.size() != 4)			// �ƽ�����Σ��޳����ı���
				continue;

			contours2.push_back(tempContour2);
		}

#ifdef DEBUG		
		Mat contours_Mat(Height, Width, CV_8UC1, Scalar(0));
		drawContours(contours_Mat, contours2, -1, Scalar(255), -1, LINE_AA);
		imshow("contours_Mat", contours_Mat);
#endif
		
		// �������ָ����Χ�ڵ�����ȡ��С��Χ���Σ��Ź�������
		vector<RotatedRect> contours_rotatedRect;
		for (int i = 0; i < contours2.size(); i++)
			contours_rotatedRect.push_back(minAreaRect(contours2[i]));

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

		// �ҵ��˾Ÿ������λ�ã���ȡ��ӦROI������ʶ��
		vector<Mat> nineRect_mat(9);	// ������ֵ�ָ�����ʹ���ľŹ������֣���Mat
		isEmpty = false;
		if (contours_rotatedRect.size() == 9) {
			sortRotatedRect(contours_rotatedRect);	// ��9����ת���ΰ���˳������
			// ����ÿ����ת���Σ��ҵ����ĸ����㣬ʹ�÷���任����任Ϊ������
			for (int i = 0; i < 9; i++) {
				vector<Point2f> p(4);
				contours_rotatedRect[i].points(p.data());
				sortPoints(p);

				Point2f srcPoints[4];
				Point2f dstPoints[4];

				srcPoints[0] = p[0] + Point2f(10, 6);
				srcPoints[1] = p[1] + Point2f(-10, 6);
				srcPoints[2] = p[2] + Point2f(-10, -4);
				srcPoints[3] = p[3] + Point2f(10, -4);

				// ���õ�һ����������ϽǺ͵�������������Ͻ�����λ������������ܣ���λ�ã��õ���Ӧ��Rect
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

				// �Ź����ڵ����������α任֮���ж���ʱ��û�����ݣ��հף�
				// ����ͨ����ͻҶ�ֵ���ж��Ƿ��������
				if (i == 0 && min_mat(dstImage) > minGrayValue) {
					cout << "no number in cells" << endl;
					isEmpty = true;
				}

				threshold(dstImage, nineRect_mat[i], 0, 255, THRESH_OTSU);
				threshold(nineRect_mat[i], nineRect_mat[i], 50, 255, THRESH_BINARY_INV);
				dilate(nineRect_mat[i], nineRect_mat[i], element2);		// ����
				deskew(nineRect_mat[i]);	// ��Ťб����
				Mat matROI_ = nineRect_mat[i].clone();
				vector<vector<Point> > contours;
				findContours(matROI_, contours, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
				if (contours.size() > 1)
					findAllContour(nineRect_mat[i], contours);
				Rect rect_ = boundingRect(contours[0]);
				Mat mattmp = nineRect_mat[i](rect_).clone();
				resize(mattmp, nineRect_mat[i], Size(40, 40));
				blur(nineRect_mat[i], nineRect_mat[i], Size(3, 3));
			}
		}
		else {
			waitKey(1);
			continue;
		}

		// �����һ��������û�����֣���������֡
		if (isEmpty)
			continue;

		/************************************************************************/
		/*                         kNNʶ��Ź���������                          */
		/************************************************************************/
		for (int i = 0; i < 9; i++) {
			hog->compute(nineRect_mat[i], descriptors);
			Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());
			Mat matCurrentChar(0, 0, CV_32F);  // findNearest�Ľ������������
			Mat m1(0, 0, CV_32F);
			Mat m2(0, 0, CV_32F);
			kNearest->findNearest(matROIFlattenedFloat, 1, matCurrentChar, m1, m2);
			nineNumber[i] = (int)matCurrentChar.at<float>(0, 0);	// ����Ź������ľŸ�����
			neighborDistance[i] = m2.at<float>(0, 0);
		}
		
		/* �жϾŹ����ʶ�������������������������ͬ���������˵����һ������ʶ�����
		 * �����ҳ��ж��ٸ�����ʶ�����
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
					cout << i + 1 << " " << nineNumber[i] << " " << neighborDistance[i] << endl;
					cout << j + 1 << " " << nineNumber[j] << " " << neighborDistance[j] << endl;
#endif
				}
			}
		}

		if (errorCount == 1) {
			/* ���ֻ��һ������ʶ������ٴ�ʹ�� kNearest->findNearest Ѱ�� 3 ������
			 * �������һ�����ֵ� 3 ������Ϊͬһ��ֵ������Ϊ��ֵ��ȷ����һ��ֵ����
			 */
			for (int i = 0; i < 2; i++) {
				hog->compute(nineRect_mat[errorPair[i]], descriptors);
				Mat matROIFlattenedFloat(1, (int)(descriptors.size()), CV_32FC1, descriptors.data());
				Mat matCurrentChar(0, 0, CV_32F);  // findNearest�Ľ������������
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

			/* ��� 3 �����ڲ���һ�����Ƚ���������������ʶ��ֵ�ÿ�����
			 * ��� neighborDistance[j] ��ֵ����˵�� nineNumber[j] ʶ�����
			 * �Ƚ��丳ֵΪ 0��Ȼ��������� nineNumber ÿ��ֵ�ú� sum
			 * ����� 45 - sum ��Ϊȱʧ��ֵ��45��1~9�ĺͣ�
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
			/* �������������ʶ����������ҵ�ʶ�������������ֵı�ţ����ҵ�������ȱʧ��������
			 * Ȼ���ٴε��� kNearest->findNearest() ��������ν� k ֵ��Ϊ 3���ҵ� ����������ھ�
			 * Ȼ������������ھ����Ƿ���������ȱʧ��ֵ��������������ʶ���������ֺ�ȱʧ��������
			 * ��Ӧ����
			 */
			int a1[2];	// ����ʶ���������ֵı��
			int b1[2];	// δʶ�������������

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
			Mat matCurrentChar(0, 0, CV_32F);  // findNearest�Ľ������������
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

		if (!foundNixieTubeArea)
			continue;

		// ����������� passwordRect �õ���Ӧ��ROI������һЩԤ����
		pw_gray = gray_img(passwordRect);
		threshold(pw_gray, pw_bin, 200, 255, THRESH_BINARY);
		erode(pw_bin, pw_bin, element2);		// ��ʴ
		dilate(pw_bin, pw_bin, element1);		// ����
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
		
#ifdef DEBUG
		imshow("frame", frame);
		imshow("password", pw_bin);
#endif

		if (ninxiTubeNumbRect.size() == 5) {
			sortRect(ninxiTubeNumbRect);
		}
		else {
			cout << "ninxiTube ERROR, ninxiTubeNumbRect.size() = " << ninxiTubeNumbRect.size() << endl;
			waitKey(1);
			continue;
		}
		
		/************************************************************************/
		/*                     kNNʶ��������������ܣ�����                        */
		/************************************************************************/
		for (int i = 0; i < 5; i++) {
			Mat matROI = pw_bin(ninxiTubeNumbRect[i]);
			resize(matROI, matROI, Size(40, 40));
			hog1->compute(matROI, descriptors1);
			Mat matROIFlattenedFloat(1, (int)(descriptors1.size()), CV_32FC1, descriptors1.data());
			Mat matCurrentChar(0, 0, CV_32F);
			kNearest1->findNearest(matROIFlattenedFloat, 1, matCurrentChar);
			password[i] = (int)matCurrentChar.at<float>(0, 0);		// �������������������
		}

		if ((nineNumber[0] * 10 + nineNumber[1]) != nineRectNumber) {
			for (int i = 0; i < 9; i++) {
				if (targetRect == nineNumber[i]) {
					Serialport1.usart3_send(static_cast<uint8_t>(i + 1));
					cout << "targetNum : " << targetRect << "-->" << i + 1 << endl;
					break;
				}
			}
			if (targetRect < 9)
				targetRect++;
			else
				targetRect = 1;
			
			nineRectNumber = nineNumber[0] * 10 + nineNumber[1];
		}
		
		// ��ӡ����������;Ź�����������
//#ifdef DEBUG
		for (int i = 0; i < 5; i++)
			cout << password[i];
		cout << "-->";

		for (int i = 0; i < 9; i++)
			cout << nineNumber[i];
		cout << endl << endl;

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
cout << "��ʱ" << time0 * 1000 << "����" << endl;
*/
