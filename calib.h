#ifndef CALIB_H
#define CALIB_H

#include <opencv2/opencv.hpp>
#include <string.h>
#include <QList>
#include <align.h>
#include <QVector>



using namespace cv;

#define interpolate(x1,y1,x2,y2,x3) (((double)y1)+((((double)x3)-((double)x1))*((((double)y2)-((double)y1))/(((double)x2)-((double)x1)))))


struct Remarkable_Corners
{

    QVector <Point3f> SX_vertical;
    QVector <Point3f> DX_vertical;
    QVector <Point2f> near_horizontal;
    QVector <float> HorLineCenterIndex;
    QVector <float>  VerLineCenterIndex;
};



typedef struct point_sort_y {
    bool operator() (Point2i pt1, Point2i pt2)
    { return (pt1.y < pt2.y);}
} pts_compare_y;


typedef struct point_sort_x {
    bool operator() (Point2i pt1, Point2i pt2)
    { return (pt1.x < pt2.x);}
} pts_compare_x;

typedef struct point_sort_z {
    bool operator() (Point3f pt1, Point3f pt2)
    { return (pt1.z < pt2.z);}
} pts_compare_z;

typedef struct point_3sort_x {
    bool operator() (Point3f pt1, Point3f pt2)
    { return (pt1.x < pt2.x);}
} pts3_compare_x;

typedef struct point_3sort_y {
    bool operator() (Point3f pt1, Point3f pt2)
    { return (pt1.y < pt2.y);}
} pts3_compare_y;

typedef struct point_3sort_z {
    bool operator() (Point3f pt1, Point3f pt2)
    { return (pt1.z < pt2.z);}
} pts3_compare_z;





struct Corner_Structure_List
{
    QVector<Point3f> delta_corrected_sorted_coordinates; // (id_mod, x_coord, y_coord)
    QVector<Point3f> orig_coordinates;    // (id_orig, x_coord, y_coord)
    QVector <Point3f> delta_corrected_idx_sorted;
    QVector <Point3f> rows_x_sorted;


};


typedef struct calib_y_3Dpoint_s
{
    Point2f  CornerPoint;
    int CornerIndex;
    int     yLineMMdistance;
} calib_y_3Dpoint_t;


typedef struct calib_z_3Dpoint_s
{
    Point2f  CornerPoint;
    int CornerIndex;
    int     zLineMMHeight;
} calib_z_3Dpoint_t;

typedef struct calib_y_Line_s
{
    QList<double>  unlinY;
    QList<int>     ZlineCoord;
} calib_y_Line_t;

typedef struct calib_z_Line_s
{
    QList<double>  unlinZ;
    QList<double>  ZlineHeight;
} calib_z_Line_t;

typedef struct line_Z_corner_s
{
    int  CenterIndex;
    int  CornerNumber;
} line_Z_corner_t;

typedef struct Candidate_corner_s
{
    Point2f  CandidateCorner_coord;
    int      CandidateCorner_intensity;
} Candidate_corner_t;

void setup_calibration(char* imgs_directory, char* imgs_filename, QString output_dir, chessboard_properties &right_cropped_chess, chessboard_properties &left_cropped_chess);


#endif // CALIB_H
