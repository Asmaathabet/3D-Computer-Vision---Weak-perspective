#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>

#include "MatrixReaderWriter.h"

using namespace cv;
using namespace std;


Mat getCoeffs(Point3f vec1, Point3f vec2) {
    Mat ret(1, 6, CV_32F);
    ret.at<float>(0, 0) = vec1.x * vec2.x;
    ret.at<float>(0, 1) = vec1.x * vec2.y + vec1.y * vec2.x;
    ret.at<float>(0, 2) = vec1.x * vec2.z + vec1.z * vec2.x;
    ret.at<float>(0, 3) = vec1.y * vec2.y;
    ret.at<float>(0, 4) = vec1.y * vec2.z + vec1.z * vec2.y;
    ret.at<float>(0, 5) = vec1.z * vec2.z;
    return ret;
}


void saveMatDouble(const char* filename, Mat matrix) {
    double* dataToSave = new double[matrix.rows * matrix.cols];
    for (int x = 0; x < matrix.cols; x++) for (int y = 0; y < matrix.rows; y++) {
        dataToSave[y * matrix.cols + x] = matrix.at<double>(y, x);
    }
    MatrixReaderWriter mrw(dataToSave, matrix.rows, matrix.cols);
    mrw.save(filename);

}


int main(int argc, char** argv)
{
    if (argc != 2)
    {
        cout << " Usage: point_file" << endl;
        return -1;
    }

    MatrixReaderWriter mtxrw(argv[1]);

    cout << mtxrw.rowNum << " " << mtxrw.columnNum << "\n";
    if ((mtxrw.rowNum < 6) || (mtxrw.columnNum < 6))
    {
        cout << "Point file format error" << std::endl;
        return -1;
    }

    double q = 1.0;
    int r = mtxrw.rowNum;
    int c = mtxrw.columnNum;

    cout << "Read: " << r << " rows and  " << c << "columns\n";


    int NUMF = r / 2;
    int NUMP = c;

    Mat data(r, c, CV_64F);

    for (int f = 0; f < NUMF; f++) {
        //Calculate center of gravity
        double centerX = 0.0;
        double centerY = 0.0;
        for (int p = 0; p < NUMP; p++) {
            double x = mtxrw.data[2 * f * NUMP + p];
            double y = mtxrw.data[(2 * f + 1) * NUMP + p];
            centerX += x;
            centerY += y;
        }
        centerX = centerX / NUMP;
        centerY = centerY / NUMP;

        for (int p = 0; p < NUMP; p++) {
            double x = mtxrw.data[2 * f * NUMP + p] - (double)centerX;
            double y = mtxrw.data[(2 * f + 1) * NUMP + p] - (double)centerY;

            data.at<double>(2 * f, p) = x;
            data.at<double>(2 * f + 1, p) = y;
        }

    }


    Mat evals, evecs;

    eigen(data * data.t(), evals, evecs);

    Mat AffMotion = evecs(cv::Rect(0, 0, 2 * NUMF, 3)).clone().t();

    cout << AffMotion.at<double>(1, 2) << endl;
    cout << evecs.at<double>(2, 1) << endl;
    cout << AffMotion.rows << endl;
    cout << AffMotion.cols << endl;
    cout << AffMotion(cv::Rect(0, 0, 3, 3)) << endl;


    Mat AffStructure = ((AffMotion.t() * AffMotion).inv()) * AffMotion.t() * data;

//*************************************
    Mat coeffMtx(2 * NUMF, 6, CV_64F); //set to 2 as we need 2 eqs
    Mat right(2 * NUMF, 1, CV_64F); //set to 2 as we need 2 eqs

//*************************************

    for (int f = 0; f < NUMF; f++) {
        Point3f vec1;
        Point3f vec2;

        vec1.x = AffMotion.at<double>(2 * f, 0);
        vec1.y = AffMotion.at<double>(2 * f, 1);
        vec1.z = AffMotion.at<double>(2 * f, 2);

        vec2.x = AffMotion.at<double>(2 * f + 1, 0);
        vec2.y = AffMotion.at<double>(2 * f + 1, 1);
        vec2.z = AffMotion.at<double>(2 * f + 1, 2);

        Mat cf1 = getCoeffs(vec1, vec1);
        Mat cf2 = getCoeffs(vec2, vec2);
        Mat cf3 = getCoeffs(vec1, vec2);


//*************************************
        Mat cfDiff = cf1 - cf2; //calculated difference to ensure they r of same length
//*************************************

        // cout << cf1 << endl;
        // cout << cf2 << endl;
        // cout << cf3 << endl;
        // cout << cfDiff << endl;


//*************************************
        for (int i = 0; i < 6; i++) {
            coeffMtx.at<double>(2 * f, i) = cfDiff.at<float>(0, i);
            coeffMtx.at<double>(2 * f + 1, i) = cf3.at<float>(0, i); }
//*************************************

        // cout << "vec1 length squared: " << vec1.dot(vec1) << endl;
        // cout << "vec2 length squared (should be same as vec1): " << vec2.dot(vec2) << endl;
        // cout << "vec1 dot vec2 (should be 0): " << vec1.dot(vec2) << endl;

        // modify from here

//*************************************
        right.at<double>(2 * f, 0) = vec1.dot(vec2); //ensuring the difference of vec's is 0 and are of equal length
        
        // right.at<double>(2 * f, 0) = vec1.dot(vec1); //also gives good result
        
        //I tried setting to 0, 1 and vec2.dot(vec2) but didnt get good results

        //***************************************
        //I believe the correct scale is length of vec1 and more close to vec1&2.  
        //***************************************
        
        right.at<double>(2 * f + 1, 0) = 0 ; //ensuring they are perpendicular
//*************************************
    }

    cout << coeffMtx << "\n";
    Mat ls = ((coeffMtx.t() * coeffMtx).inv()) * coeffMtx.t() * right;
    Mat L(3, 3, CV_64F);

    L.at<double>(0, 0) = ls.at<double>(0, 0);
    L.at<double>(0, 1) = L.at<double>(1, 0) = ls.at<double>(1, 0);
    L.at<double>(0, 2) = L.at<double>(2, 0) = ls.at<double>(2, 0);
    L.at<double>(1, 1) = ls.at<double>(3, 0);
    L.at<double>(1, 2) = L.at<double>(2, 1) = ls.at<double>(4, 0);
    L.at<double>(2, 2) = ls.at<double>(5, 0);
    Mat U(3, 3, CV_64F);
    Mat S(3, 3, CV_64F);
    Mat V(3, 3, CV_64F);

    cout << "Hello3\n";
    SVD::compute(L, S, U, V);

    cout << L << endl;

    cout << S << endl;
    cout << U << endl;
    cout << V << endl;

    cout << "Hello4\n";
    Mat sqrtS = Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 3; i++) sqrtS.at<double>(i, i) = sqrt(S.at<double>(i));
    cout << sqrtS << endl;
    cout << S << endl;

    Mat Q = U * sqrtS;

    Mat Motion = AffMotion * Q;
    Mat Structure = Q.inv() * AffStructure;

    saveMatDouble("AffStructure.mat", AffStructure.t());
    saveMatDouble("AffMotion.mat", AffMotion);

    saveMatDouble("Structure.mat", Structure.t());
    saveMatDouble("Structure.xyz", Structure.t());
    saveMatDouble("Motion.mat", Motion);

    //    cout <<AffMotion<<endl;


    return 0;
}
