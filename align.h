#ifndef ALIGN
#define ALIGN


#include <opencv2/opencv.hpp>
#include <string.h>
#include <QList>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <QList>
#include <QFile>
#include <QDebug>
#include <QMessageBox>

#include <math.h>
#include <vector>
#include <algorithm>
#include <QElapsedTimer>
#include <numeric>
#include <functional>
#include <iterator>

using namespace cv;
using namespace std;

 struct sources

{
Mat grayscale;
Mat color;
Mat bin;
};


 struct cross_hair
 {
     bool vertical;
     bool horizontal;
 };

 struct chessboard_properties

{
int mean_top_limit;
int mean_bottom_limit;
int hor_line_quote;
int ver_line_quote;
Point2i corner_up_left;
Point2i corner_down_right;
Point2i calibration_point;

vector <Point2f> top_contour;
vector <Point2i> deltas;
vector <Point2i> y_center_deltas;
vector <Point2i> x_center_deltas;
QVector <Point2f> horizontal_pointlike;
QVector <Point2f> vertical_pointlike;

cross_hair crosshair;
cross_hair cropped_crosshair;

};



typedef struct points_sort_y {
    bool operator() (Point2i pt1, Point2i pt2)
    { return (pt1.y < pt2.y);}
} point_compare_y;


typedef struct points_sort_x {
    bool operator() (Point2i pt1, Point2i pt2)
    { return (pt1.x < pt2.x);}
} point_compare_x;




int extract_vertical_black_line  (chessboard_properties &chess, sources &src);

int extract_chess_limits (chessboard_properties &chess, sources &src, bool above, bool below);

int extract_relative_dimensions (chessboard_properties &chess, sources &src, int col_start, int col_final );

int extract_horizontal_black_line (chessboard_properties &chess, sources &src);

int extract_x_correction (chessboard_properties &chess, sources &src);


void align(char* imgs_filename, chessboard_properties &left_chess, chessboard_properties &right_chess, chessboard_properties &left_chess_cropped, chessboard_properties &right_chess_cropped);

void create_transformed_vec2f(int R, QVector <Point2f> &vec_dst, chessboard_properties &chess_properties, sources &img_src );


#endif // ALIGN

