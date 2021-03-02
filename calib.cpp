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
#include <calib.h>
#include <align.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <QElapsedTimer>
#include <QVector>
#include <numeric>
#include <functional>
#include <iterator>


// TO DO BARICENTROIDI DEI QUADRATI ADIACENTI AI LATI VERTICALI: BIANCO A PARTIRE DAL BORDO


// TO DO: filtro riga per riga dopo sorting

// TO DO: raccogliere tutto ciò che riguarda i corners in una struttura "Corners"

// TO DO: riscrivi baricentroids con differenza in coordinata x e y anzichè

/*
 *
 *
 * filtro riga per riga su righe già ruotate per individuare corner mancanti o in eccesso;

confronto di ogni linea non ruotata con la linea bianca superiore per controllo?

ottimizza baricentroidi;

scrivi estrapolatore linee aggiuntive z;

!!!!!!!!!!!
LINEA ORIZZONTALE COME INSIEME DI PUNTI CHE HANNO IL NERO IN UN INTORNO E LA CUI SOMMA DELLE DISTANZE DAL BORDO ALTO E BASSO è COSTANTE.
*/




using namespace std;
using namespace cv;


// Chessboard and Image Parameters

#define ImgWidth 2000
#define ImgHeight 600

#define square_size_mm 135 //128 for real image mm; physical dimension of chessboard square
#define ZthrHeight_pix 5 //pixel; to distinguish among Z lines for sorting
#define ZthrXDiff_pix 950 //pixel; to distinguish among Z lines in X difference


// GoodFeaturesToTrack params

#define quality_lev 0.03
#define blockSize 6
#define max_corners 1000
#define min_distance 15
#define Par_k 0.03
#define useHarrisDetector false


// Corner Filtering algorithm parameter
#define neighbour_toll 12//5
#define neighbour_toll_refined 0


// Find center algorithm parameters
#define PercBlack 4 //percentage of number of black pixels higher wrt white pixels to find the single central all-black line

#define HorProxCenterBlackLine 40 // number of pixels maximum within which the corner must to be found close to the central all-black horizontal line
#define VerProxCenterBlackLine 50 // number of pixels maximum within which the corner must to be found close to the central all-black horizontal line
#define LimitInHorBlackLine 70 // number of pixels excluded at top and bottom to find the center horizontal black line
#define LimitInVerBlackLine 100 // number of pixels excluded at left and right to find the center vertical black line


// Storing original images

Mat img_grayscale, img_color;


// Storing data to produce the tables

QList < calib_y_Line_t > tableY;
QList < calib_z_Line_t > tableZ;


template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}


// Points coordinate comparator to ease the implementation of std::sort
bool operator == (const Point2i& lhs, const Point2i& rhs)
{
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

pts_compare_y points_compare_y;
pts_compare_x points_compare_x;
pts_compare_z points_compare_z;

// BARICENTROID FILTER
//Remove corners who's coordinated are too close to the baricenter of the rectangle approximating each white parallelogramS

void baricentroids_filter( chessboard_properties &right_chess_cropped, QVector <Point2f> &corners_to_sort, QVector <Point2f> &corners_cleaned, int rows_offset, int cols_offset, cv::Mat &raw_image, cv::Mat &img_color_enl, bool display_barirectangles, double energy)

{
    if(energy < 0 || energy > 1.0)
    {
        cout << "Energy has to be >0 and <1" << endl;
        std::abort();
    }

    QVector <Point2i> baricentroids_white;
    QVector <Point2i> pointz;
    int scale_factor=2;
    Mat refined_binary_white_enh;

    // The algorithm will work with low-thresholded (white enhanced) version of the image

    cv::threshold(raw_image, refined_binary_white_enh, 50, 255, THRESH_BINARY);

    int scope_rows=rows_offset;
    int scope_cols=cols_offset;
    int h_ctrl, v_ctrl;
    vector <float> horL, horR;
    vector <float> verUp, verDw;

    // Sorting based on the horizontal black line quote

    for (int r = scope_rows; r < (refined_binary_white_enh.rows - scope_rows) ; r++)
    {
        for (int c=scope_cols; c < refined_binary_white_enh.cols - scope_cols; c++)
        {
            // SQUARES
            if (r > right_chess_cropped.hor_line_quote * scale_factor)
            {
                v_ctrl=h_ctrl=13;
            }

            // RECTANGLES
            else

            {
                v_ctrl=10;
                h_ctrl=38;
            }
            // Checking the color on the points at the 4 corners of the square(lower half of the chessboard)/
            // rectangle(upper_half) centered on the candidate baricentroidand
            // the color of the baricentroid himself


            if(refined_binary_white_enh.at<uchar>(r-v_ctrl, c-h_ctrl) > 254 &
                    refined_binary_white_enh.at<uchar>(r+v_ctrl, c+h_ctrl) > 254 &
                    refined_binary_white_enh.at<uchar>(r-v_ctrl, c+h_ctrl) > 254 &
                    refined_binary_white_enh.at<uchar>(r+v_ctrl, c-h_ctrl) > 254 &
                    refined_binary_white_enh.at<uchar>(r,c)> 254)


            {
                for(int h=0; h< scope_cols; h++ )
                {
                    if (refined_binary_white_enh.at<uchar>(r, c - h) > 254)
                    {
                        horL.push_back(refined_binary_white_enh.at<uchar>(r, c - h));
                    }
                    else continue;
                }
                for(int h=0; h< scope_cols; h++ )
                {
                    if (refined_binary_white_enh.at<uchar>(r, c + h) > 254)
                    {
                        horR.push_back(refined_binary_white_enh.at<uchar>(r, c + h));
                    }
                    else continue;
                }
                for(int h=0; h< scope_rows; h++ )
                {
                    if (refined_binary_white_enh.at<uchar>(r+h , c) > 254)
                    {
                        verUp.push_back(refined_binary_white_enh.at<uchar>(r+h, c));
                    }
                    else continue;
                }
                for(int h=0; h< scope_rows; h++ )
                {
                    if (refined_binary_white_enh.at<uchar>(r- h, c) > 254)
                    {
                        verDw.push_back(refined_binary_white_enh.at<uchar>(r-h, c));
                    }
                    else continue;
                }
            }


            if ((horL.size() - horR.size() <3) && (verUp.size() - verDw.size() <3)
                    && (horL.size() >10) && (horR.size() >10) && (verDw.size() > 10) && (verUp.size() >10) )
            {
                baricentroids_white.push_back(Point2f(c, r));
            }

            else  if ((horL.size() - horR.size() <2) && (verUp.size() - verDw.size() <2)
                      && (horL.size() >10) && (horR.size() >10) && (verDw.size() > 10) && (verUp.size() >10) )
            {
                baricentroids_white.pop_back();
                baricentroids_white.pop_back();

                baricentroids_white.push_back(Point2f(c, r));

            }
            else if ((horL.size() - horR.size() ==0) && (verUp.size() - verDw.size() ==0)
                     && (horL.size() >10) && (horR.size() >10) && (verDw.size() > 10) && (verUp.size() >10))
            {
                baricentroids_white.pop_back();
                baricentroids_white.pop_back();
                baricentroids_white.push_back(Point2f(c, r));
            }

            horL.clear();
            horR.clear();
            verDw.clear();
            verUp.clear();

        }
    }


    for( int i = 0; i < baricentroids_white.size(); i++ )
    {
        circle( img_color_enl, Point( (int)baricentroids_white.at(i).x, (int)baricentroids_white.at(i).y ) , 2,  CV_RGB(0,255,0), 2, 8, 0 );
    }

    cv::imshow("baricentroids", img_color_enl);
    waitKey(0);


    // Sorting Baricentroids between upper-half of the chessboard and lower-half, rect and square respectively

    QVector <Point> rect_baricentroids, square_baricentroids;

    for (int i=0; i< baricentroids_white.size(); i++)
    {
        if (baricentroids_white.at(i).x < right_chess_cropped.hor_line_quote*scale_factor)
        {
            rect_baricentroids.push_back(baricentroids_white.at(i));
        }
        else
        {
            square_baricentroids.push_back(baricentroids_white.at(i));
        }
    }

    // Cleaning corner list with baricentroids

    vector <Point2i> pointz_white_ver, pointz_white_hor;

    for (int j=0; j < baricentroids_white.size(); j++)
    {

        int horizontal_tolerance=0;

        for (int n=-scope_cols +10; n< scope_cols -10; n++)
        {
            if (refined_binary_white_enh.at<uchar>(baricentroids_white.at(j).y + n , baricentroids_white.at(j).x) > 254 )
            {
                horizontal_tolerance+=1;
            }
        }

        int vertical_tolerance=0;

        for (int m=-scope_rows; m< scope_rows; m++)
        {
            if (refined_binary_white_enh.at<uchar>(baricentroids_white.at(j).y , baricentroids_white.at(j).x + m) > 254 )
            {
                vertical_tolerance+=1;
            }
        }

        // COORDINATE (righe, colonne) - (Y, X)

        // Default v: .5, .7
        // Default h: .1, .2

        QVector <double> v_weights, h_weights;

        v_weights.append(.8*energy);    //top and bottom areas, bottom half
        v_weights.append( 1.4  *energy);   // top and bottom areas, top half
        h_weights.append(.25*energy); // left and right areas, bottom half
        h_weights.append(.1*energy); // left and right areas, top half


        // Vertical rectangles centered on the baricentroid of white squares (GREEN)

        for (int altezza=0; altezza < 10 ; altezza++)
        {

            if (baricentroids_white.at(j).y > right_chess_cropped.hor_line_quote*scale_factor)
            {
                //Sotto
                for (int larghezza= -floor(v_weights.at(0)*horizontal_tolerance) -2; larghezza< floor(v_weights.at(0)*horizontal_tolerance) +2 ; larghezza++ )
                {
                    pointz_white_ver.push_back(Point2i(baricentroids_white.at(j).y +15 + altezza, baricentroids_white.at(j).x + larghezza));
                }
                //Sopra

                for (int larghezza= -floor(v_weights.at(0)*horizontal_tolerance) -2; larghezza< floor(v_weights.at(0)*horizontal_tolerance) +2 ; larghezza++ )
                {
                    pointz_white_ver.push_back(Point2i(baricentroids_white.at(j).y - 15 - altezza, baricentroids_white.at(j).x + larghezza));
                }

            }

            else
            {
                //Sotto
                for (int larghezza= -floor(v_weights.at(1)*horizontal_tolerance) ; larghezza< floor(v_weights.at(1)*horizontal_tolerance)  ; larghezza++ )
                {
                    pointz_white_ver.push_back(Point2i(baricentroids_white.at(j).y +8 + altezza, baricentroids_white.at(j).x + larghezza));
                }
                //Sopra

                for (int larghezza= -floor(v_weights.at(1)*horizontal_tolerance) ; larghezza < floor(v_weights.at(1)*horizontal_tolerance) ; larghezza++ )
                {
                    pointz_white_ver.push_back(Point2i(baricentroids_white.at(j).y - 8 - altezza, baricentroids_white.at(j).x + larghezza));
                }

            }

        }

        // Horizontal rectangles centered on the baricentroid of white squares (BLUE)
        if (baricentroids_white.at(j).y > right_chess_cropped.hor_line_quote*scale_factor)
        {// Upper half
            for (int larghezza=0; larghezza < 7 ; larghezza++)
            {
                //Destra
                for (int altezza=-floor(h_weights.at(0)*vertical_tolerance); altezza < floor(h_weights.at(0)*vertical_tolerance); altezza++)
                {
                    pointz_white_hor.push_back(Point2i(baricentroids_white.at(j).y + altezza, baricentroids_white.at(j).x +33 + larghezza));
                }

                // Sinistra
                for (int altezza=-floor(h_weights.at(0)*vertical_tolerance); altezza < floor(h_weights.at(0)*vertical_tolerance); altezza++)
                {
                    pointz_white_hor.push_back(Point2i(baricentroids_white.at(j).y + altezza, baricentroids_white.at(j).x - 33 - larghezza));
                }
            }
        }

        else
        {//lower half
            for (int larghezza=0; larghezza < 10 ; larghezza++)
            {
                //Destra
                for (int altezza=-floor(h_weights.at(1)*vertical_tolerance); altezza < floor(h_weights.at(1)*vertical_tolerance); altezza++)
                {
                    pointz_white_hor.push_back(Point2i(baricentroids_white.at(j).y + altezza, baricentroids_white.at(j).x +33 + larghezza));
                }

                // Sinistra
                for (int altezza=-floor(h_weights.at(1)*vertical_tolerance); altezza < floor(h_weights.at(1)*vertical_tolerance); altezza++)
                {
                    pointz_white_hor.push_back(Point2i(baricentroids_white.at(j).y + altezza, baricentroids_white.at(j).x - 33 - larghezza));
                }
            }
        }
    }


    for (int i =0; i< pointz_white_hor.size(); i++)
    {
        pointz.push_back(pointz_white_hor.at(i));
    }


    for (int i =0; i< pointz_white_ver.size(); i++)
    {
        pointz.push_back(pointz_white_ver.at(i));
    }

    cout << "Removing bad corners tangent to the white squares..." << endl;


    //Drawing the bari-rectangles

    if (display_barirectangles== true)
    {
        for( int i = 0; i < pointz_white_ver.size(); i++ ) ///GREEN
        {
            circle( img_color_enl, Point( (int)pointz_white_ver.at(i).y, (int)pointz_white_ver.at(i).x ) , 1,  CV_RGB(0,255,0), 2, 8, 0 );
        }

        for( int i = 0; i < pointz_white_hor.size(); i++ ) /// BLUE
        {
            circle( img_color_enl, Point( (int)pointz_white_hor.at(i).y, (int)pointz_white_hor.at(i).x ) , 1,  CV_RGB(0,0,255), 2, 8, 0 );
        }


        imshow("Positions of the baricentroids", img_color_enl );
        waitKey(0);

    }
    //Removing corners from the list whose coordinates belongs to the bari-rectangles


    for (int i=0; i< corners_to_sort.size(); i++)
    {
        for (int k=0; k< pointz.size(); k++)
        {
            if (corners_to_sort.at(i) == Point2f (pointz.at(k).y, pointz.at(k).x) )
            {
                corners_to_sort.remove(i);
                break;

            }
            else continue;
        }
    }

    for (int corner=0; corner< corners_to_sort.size(); corner++)
    {
        corners_cleaned.push_back(corners_to_sort.at(corner));
    }

    return;
}



/// DUST FILTER
///
/// Filter corners not on the edges with a control on the medium color of pixels contained on the (n,n) block centered on the corner
void dust_filter(QVector <Point2f> &corners, cv::Mat &refined, cv::Mat &img_color_enl, bool display_removed_corners )
{
    int mean_check;
    int o=0;
    int p=0;
    vector <float> corner_neighbours;
    vector <Point2f> corners_in_white, corners_in_black;

    int count=0;
    int mean_refined=0;

    for (int r=1; r<refined.rows; r++)
    {
        for (int c=1; c< refined.cols; c++)
        {
            count+=refined.at<uchar>(r, c);

        }
    }
    mean_refined= floor(count/refined.total());

    // Binarized image with total color mean used as threshold

    Mat refined_binary;

    threshold(refined, refined_binary, mean_refined, 255, THRESH_BINARY);


    for (int a=0; a< corners.size(); a++)
    {
        Point2f centro(corners.at(a).x, corners.at(a).y);

        if (corners.at(a).x > 5 && corners.at(a).x <refined_binary.cols -3 && corners.at(a).y >0 && corners.at(a).y < refined_binary.rows)
        {
            for (int r=-4; r<=4; r++)
            {
                for (int c=-4; c<=4  ; c++)
                {
                    corner_neighbours.push_back((refined_binary.at<uchar>(centro.y + r, centro.x + c)));
                }
            }
            mean_check= floor(std::accumulate(corner_neighbours.begin(), corner_neighbours.end(), 0)/(corner_neighbours.size()));


            if (mean_check > 230)
            {
                o++;
                corners.remove(a);
                corners_in_white.push_back(centro);
            }

            if (mean_check < 5)
            {
                p++;
                corners.remove(a);
                corners_in_black.push_back(centro);
            }

            corner_neighbours.clear();
        }
    }


    cout << corners_in_black.size() << " corners are contained in black areas and " << corners_in_white.size() << "in white ones and has been removed. Both drawn in blue." << endl;

    if (display_removed_corners=true)
    {
        for( int i = 0; i < corners_in_black.size(); i++ )
        {
            circle( img_color_enl, Point( (int)corners_in_black.at(i).x, (int)corners_in_black.at(i).y ) , 5,  CV_RGB(255,0,0), 2, 8, 0 );
        }

        for( int i = 0; i < corners_in_white.size(); i++ )
        {
            circle( img_color_enl, Point( (int)corners_in_white.at(i).x, (int)corners_in_white.at(i).y ) , 5,  CV_RGB(255,0,0), 2, 8, 0 );
        }

        imshow("Corners removed with DUST filter", img_color_enl );
        waitKey(0);
    }

    // Releasing the resources

    corners_in_black.clear();
    corners_in_white.clear();
    corner_neighbours.clear();
}


//TOP BORDER FILTER
//
// Filter corners closer that a distance (maxdistance) to the top white line. Euclidean norm used.

void  top_border_filter (cv::Mat &img_color_enl, chessboard_properties &chess_cropped, QVector <Point2f> &corners_to_sort, double max_distance, bool display_results)
{
    //https://docs.opencv.org/2.4/modules/core/doc/operations_on_arrays.html?highlight=cv2.norm#norm

    QVector <Point2f>  top_removed;

    for (int c=0; c < corners_to_sort.size(); c++)

    {   if (corners_to_sort.at(c).y < 200)
        {
            for (int b=0; b < chess_cropped.top_contour.size(); b++)
            {
                if ( cv::norm( corners_to_sort.at(c) - chess_cropped.top_contour.at(b)) < 30 )
                {
                    top_removed.push_back(corners_to_sort.at(c));
                    break;
                }
                else continue;
            }
        }
        else continue;
    }

    // Removing the corners from the main list.
    for (int a=0; a < corners_to_sort.size(); a++)
    {
        for (int m=0; m< top_removed.size(); m++)
        {
            if (corners_to_sort.at(a)== top_removed.at(m))
            {
                corners_to_sort.remove(a);
            }
            else continue;
        }

    }

    if (display_results)
    {
        for (int i=0; i < top_removed.size(); i++)
        {
            circle( img_color_enl, Point( (int)top_removed.at(i).x, (int)top_removed.at(i).y ) , 5,  CV_RGB(255,255,0), 2, 8, 0 );

        }

        for (int i=0; i < chess_cropped.top_contour.size(); i++)
        {
            circle( img_color_enl, Point( (int)chess_cropped.top_contour.at(i).x, (int)chess_cropped.top_contour.at(i).y ) , 1,  CV_RGB(255,255,0), 2, 8, 0 );

        }

        imshow("Corners removed with TOP filter", img_color_enl );
        waitKey(0);
    }


    return ;
}



// Function that produces a scaled "sources" structure
void apply_scale_to_src(sources &src, sources &scaled_src , int scale_factor)
{

    resize(src.grayscale, scaled_src.grayscale, Size(), scale_factor, scale_factor, INTER_LINEAR);
    resize(src.color, scaled_src.color, Size(), scale_factor, scale_factor,  INTER_LINEAR);
    threshold(scaled_src.grayscale, scaled_src.bin, 70, 255, THRESH_BINARY);

}



//Function that apply the extracted deltas of the image to a QVector of floats
void apply_deltas_to_sparse_vec2f(QVector <Point2f> &vec_src, QVector <Point2f> &vec_dst, chessboard_properties &chess_properties, sources &img_src, bool display_results)

{


    for (int n=0; n< vec_src.size(); n++)
    {
        for (int c=0; c< img_src.grayscale.cols; c++)
        {

            if (vec_src.at(n).x == chess_properties.deltas.at(c).x)
            {

                vec_dst.push_back(Point2f(vec_src.at(n).x, vec_src.at(n).y + chess_properties.deltas.at(c).y));

            }

            else continue;
        }
    }

    if (display_results)
    {
        Mat bribba;
        cv::Mat brabbabbbababbaba = Mat::zeros(img_src.grayscale.size(), CV_32FC1);
        resize(brabbabbbababbaba, bribba, Size(), 2, 2, INTER_LINEAR );

        for (int i=0;  i< vec_dst.size(); i ++)
        {
            circle(img_src.color,Point( (int)vec_dst.at(i).x, (int)vec_dst.at(i).y ) , 2,  CV_RGB(255,255,0), 2, 8, 0 );
        }
        imshow("ASGABADAVAI", img_src.color);
        waitKey(0);
    }

    return ;

}




void apply_deltas_to_sparse_vec3f(QVector <Point3f> &vec_src, QVector <Point3f> &vec_dst, chessboard_properties &chess_properties, sources &img_src)
{

    for (int n=0; n< vec_src.size(); n++)
    {
        for (int c=0; c< img_src.grayscale.cols; c++)
        {

            if (vec_src.at(n).y == chess_properties.deltas.at(c).x)
            {

                vec_dst.push_back(Point3f(vec_src.at(n).x, vec_src.at(n).y, vec_src.at(n).z + chess_properties.deltas.at(c).y));

            }

            else continue;
        }
    }
    return;
}




void apply_center_rotation_to_vec3f(QVector <Point3f> &vec_src, QVector <Point3f> &vec_dst, chessboard_properties &chess_properties, sources &img_src)
{

    for (int n=0; n< vec_src.size(); n++)
    {   Point3f aux;

        for (int c=0; c< img_src.grayscale.cols; c++)
        {
            if (vec_src.at(n).y == chess_properties.deltas.at(c).x)
            {
                aux.z = vec_src.at(n).z - chess_properties.x_center_deltas.at(c).y;
                aux.x = vec_src.at(n).x;
            }

            else continue;
        }

        for (int c=0; c< img_src.grayscale.rows; c++)
        {
            if(vec_src.at(n).z == chess_properties.y_center_deltas.at(c).x)
            {

                int ULLA=  vec_src.at(n).y - chess_properties.y_center_deltas.at(c).y;
                aux.y = ULLA;
                vec_dst.push_back(aux);
                break;
            }

            else continue;
        }

    }

    return;
}



Point2f retrieve_orig_coord_from_idx (Corner_Structure_List &corner_structure_list, int idx)
{
    Point2f dst;
    for (int i=0; i< corner_structure_list.orig_coordinates.size(); i++)
    {
        if (corner_structure_list.orig_coordinates.at(i).x == idx)

        {   dst.x=corner_structure_list.orig_coordinates.at(i).y;
            dst.y=corner_structure_list.orig_coordinates.at(i).z;
        }


    }

    return dst;

}



// Function that create a list of Point3f(idx, x, y) and a list of Point3f(idx, x', y') where the coordinated has been transformed
// via del deltas of the image.

Corner_Structure_List create_corner_structure_list(QVector<Point2f> &cleaned_corners_list,
                                                   chessboard_properties &chess_properties, sources &img_src)
{
    Corner_Structure_List corner_structure_list;


    for (int i=0; i<cleaned_corners_list.size(); i++)
    {
        corner_structure_list.orig_coordinates.push_back(Point3f(i, cleaned_corners_list.at(i).x, cleaned_corners_list.at(i).y));
    }

    apply_center_rotation_to_vec3f(corner_structure_list.orig_coordinates, corner_structure_list.delta_corrected_sorted_coordinates, chess_properties, img_src);




    return corner_structure_list;
}

void apply_corner_mask(QVector <Point2f> &corners, chessboard_properties &chess_prop, sources &src, int max_distance_from_top_white_border)
{

    QVector <Point2f> submerged;
    for (int i=0; i < corners.size(); i++)
    {
        for (int j=0; j< chess_prop.top_contour.size(); j++)
        {
            if (corners.at(i).x == chess_prop.top_contour.at(j).x &&
                    cv::norm(corners.at(i) - chess_prop.top_contour.at(j)) > max_distance_from_top_white_border)
            {
                submerged.push_back(corners.at(i));
            }
            else
            {
                continue;
            }

        }
    }


    for (int k=0; k< corners.size(); k++)
    {
        if (corners.at(k).x < 10 || corners.at(k).x > src.grayscale.cols -10)
        {
            submerged.push_back(corners.at(k));
        }
    }

    for (int b=0; b <submerged.size(); b++)
    {
        for (int a=0; a< corners.size(); a++)
        {
            if (submerged.at(b).x==corners.at(a).x && submerged.at(b).y == corners.at(a).y)
            {
                corners.remove(a);

            }

            else continue;
        }
    }
    return;
}



void setup_calibration(char* imgs_directory, char* imgs_filename, QString output_dir, chessboard_properties &right_chess_cropped, chessboard_properties &left_chess_cropped )
{

    /// TIMER START

    QElapsedTimer timer;
    timer.start();

    // To store final corners coordinates

    QVector< Point2f > corners;

    // To store the processed source image

    sources src, scaled_src;

    chessboard_properties scaled_chess;

    // Corners and then refined corners coordinates will be stored on this vector:

    vector<Point2f> crns;

    // Chessboard Image Upload

    string img_file = "/home/ale/Desktop/acquisizioni_v2_0709/tilted_artificiali/synth_cropped/right_tilted_cropped.BMP";

    src.color = imread(img_file, IMREAD_COLOR);
    src.grayscale = imread(img_file, IMREAD_GRAYSCALE);
    threshold(src.grayscale, src.bin, 70, 255, THRESH_BINARY);


    // To ease the analysis, every element of the Mat representing the image will be linearly multiplied for a scale factor:

    int scale_factor ;
    scale_factor=2;

    // Clean source

    Mat clean_scaled_src;
    resize(src.color, clean_scaled_src, Size(), 2, 2, INTER_LINEAR);


    apply_scale_to_src(src, scaled_src, 2);


    // Enlarged original (color) images that will be used to show step-to-step results.


    Mat refined= scaled_src.grayscale.clone();

    Candidate_corner_t CornerCand;

    QVector<Candidate_corner_t> CandidateCornList;


    // Setting the mask for the corner detector

    Mat mask = Mat::zeros(refined.size(), CV_8U);


    //// MAKE THE MASK ADAPTIVE


    Mat safe_detection_roi(mask, cv::Rect(0, 30,refined.cols, refined.rows-100 ));
    safe_detection_roi = Scalar(255);


    // Set the needed parameters to find the refined corners with cornersubpix

    Size winSize = Size( 10, 10);
    Size zeroZone = Size( -1, -1 );
    TermCriteria criteria = TermCriteria( TermCriteria::EPS + TermCriteria::MAX_ITER , 70, 0.0001);


    // WHOLE-IMAGE COLOR MEAN TO BE USED AS THRESHOLD

    int count=0;
    int mean_refined=0;

    for (int r=1; r<refined.rows; r++)
    {
        for (int c=1; c< refined.cols; c++)
        {
            count+=refined.at<uchar>(r, c);

        }
    }
    mean_refined= floor(count/refined.total());


    // Detecting corners with GFTT

    goodFeaturesToTrack( refined, crns, max_corners, quality_lev, min_distance, mask, blockSize, useHarrisDetector , Par_k );


    // Refined corner locations:

    cornerSubPix(refined, crns, winSize, zeroZone, criteria);


    std::sort(crns.begin(), crns.end(), points_compare_y);


    // Corners List creation

    for( int j = 0; j < crns.size(); j++ )

    {
        CornerCand.CandidateCorner_coord = (Point2i) crns.at(j);
        CornerCand.CandidateCorner_intensity = 200; // Standard, remaining from the harris detector epoch
        CandidateCornList.append(CornerCand);
    }

    /// Positional filter


    int x = 0;
    int y = 0;

    QList <int> addedIndex;

    for(int i = 0; i< CandidateCornList.size(); i++ )
    {
        QList <int> MacroPindexList;

        if((fabs((CandidateCornList.at(i).CandidateCorner_coord.x)-x) > neighbour_toll || fabs((CandidateCornList.at(i).CandidateCorner_coord.y)-y) > neighbour_toll) && !addedIndex.contains(i) )
        {
            x = CandidateCornList.at(i).CandidateCorner_coord.x;
            y = CandidateCornList.at(i).CandidateCorner_coord.y;
            MacroPindexList.append(i);
            addedIndex.append(i);

            for (int j = 1; (i+j) < CandidateCornList.size(); j++)
            {
                if((fabs((CandidateCornList.at(i+j).CandidateCorner_coord.x)-x) < neighbour_toll && fabs((CandidateCornList.at(i+j).CandidateCorner_coord.y)-y) < neighbour_toll) && !addedIndex.contains(i+j))
                {
                    MacroPindexList.append(i+j);
                    addedIndex.append(i+j);
                }
            }
            int MaxInt = 0;
            int indMaxInt =0;

            for( int t = 0; t < MacroPindexList.size(); t++)
            {
                if(CandidateCornList.at(MacroPindexList.at(t)).CandidateCorner_intensity > MaxInt)
                {
                    MaxInt =  CandidateCornList.at(MacroPindexList.at(t)).CandidateCorner_intensity;
                    indMaxInt = MacroPindexList.at(t);
                }
            }
            corners.append(CandidateCornList.at(indMaxInt).CandidateCorner_coord);
        }
    }


    for( int i = 0; i < corners.size(); i++ )
    {
        circle( scaled_src.color, Point( (int)corners.at(i).x, (int)corners.at(i).y ) , 5,  CV_RGB(0,0,255), 2, 8, 0 );
    }


    cv::imshow("Total corners" , scaled_src.color);
    waitKey(0);

    /*
    Point2f scambio = Point2f(0,0);
    scambio = Point2f(0,0);


    //Ordinamento NON FUNZIONA PIU'


    int in =0;
    int fin =1;

    for (int i = 0; i < corners.size(); i++)
    {
        if(((i+1) < corners.size()) && fabs((int)corners.at(i).y - (int)corners.at(i+1).y) <= (3))

        {
            fin++;
        }
        else
        {
            int j = fin-1;
            for (int i = in; i < fin; i++)
            {
                for (int k = in; k < j; k++)
                {
                    if (corners[k].x > corners[k+1].x) //Se il numero della posizione attuale e' maggiore del successivo scambio i valori
                    {
                        scambio = corners[k+1];
                        corners[k+1] = corners[k];
                        corners[k] = scambio;
                    }
                }

                j--;
            }
            in = fin;
            if(fin < corners.size()-1)
            {
                fin++;
            }
        }
    }

*/
    extract_relative_dimensions(scaled_chess, scaled_src, 0, scaled_src.grayscale.cols);

    extract_chess_limits(scaled_chess, scaled_src, true, false);

    extract_vertical_black_line(scaled_chess, scaled_src);

    extract_horizontal_black_line(scaled_chess, scaled_src);

    extract_x_correction(scaled_chess, scaled_src);


    ///
    /// CORNER FILTERING
    ///
    ///
    ///
    ///
    cout << "Applying mask filter..." << endl;

    apply_corner_mask(corners, scaled_chess,scaled_src, 400);


    for( int i = 0; i < corners.size(); i++ )
    {
        circle( scaled_src.color, Point( (int)corners.at(i).x, (int)corners.at(i).y ) , 5,  CV_RGB(0,255,0), 2, 8, 0 );
    }

    imshow("Masked corners", scaled_src.color );
    waitKey(0);


    cout << "Applying dust filter..." << endl;

    //DUST FILTER

    dust_filter(corners, refined, scaled_src.color, true );


    /// TOP WHITE BORDER FILTER
    ///
    cout << "Applying top border filter..." << endl;

    top_border_filter( scaled_src.color, scaled_chess, corners, 40, false);


    //BARICENTROID FILTER


    cout << "Applying baricentroid filter..." << endl;


    QVector <Point2f> cleaned_corners;


    baricentroids_filter( right_chess_cropped, corners, cleaned_corners, 30, 40, refined, scaled_src.color, false , 0.5F);

    int cornering_time = timer.elapsed();


    cout << "...done!" << "Time spent in cornering: " << cornering_time << endl;

    // TIMER STOP



    for( int i = 0; i < cleaned_corners.size(); i++ )
    {
        circle( scaled_src.color, Point( (int)cleaned_corners.at(i).x, (int)cleaned_corners.at(i).y ) , 5,  CV_RGB(0,255,0), 2, 8, 0 );
    }

    imshow("Final corner positions", scaled_src.color );
    waitKey(0);
    destroyAllWindows();



    /// Corner_Structure_List:
    /// Corner_Structure_List.delta_corrected_sorted_coordinates: (idx, x', y')
    /// Corner_Structure_List.orig_coordinates: (idx, x, y)

    Corner_Structure_List corner_structure_list = create_corner_structure_list(cleaned_corners, scaled_chess, scaled_src);
    pts3_compare_x POINTS3_COMPARE_X;
    pts3_compare_y POINTS3_COMPARE_Y;
    pts3_compare_z POINTS3_COMPARE_Z;

    for( int i = 0; i < corner_structure_list.delta_corrected_sorted_coordinates.size(); i++ )
    {
        circle( clean_scaled_src, Point( (int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).y,
                                         (int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).z ) , 5,  CV_RGB(0,255,0), 2, 8, 0 );
    }

    imshow("Final corner positions", clean_scaled_src );
    waitKey(0);

    Remarkable_Corners remarkable_corners;

    // Only a copy of the color source to show final results.
    Mat img_FINAL;
    resize(scaled_src.color, img_FINAL, Size(), 1, 1, INTER_LINEAR);





    // Locate corners to the right of the vertical and on top of the horizontal line


    // Horizontal
    for( int i = 0; i < corner_structure_list.orig_coordinates.size(); i++ )
    {
        for (int k=0; k < scaled_chess.horizontal_pointlike.size(); k++)
        {
            if(  cv::norm(scaled_chess.horizontal_pointlike.at(k) -
                          Point2f(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z)) <= HorProxCenterBlackLine &&

                 ( (int)corner_structure_list.orig_coordinates.at(i).z - scaled_chess.horizontal_pointlike.at(k).y ) >0 )
            {
                remarkable_corners.HorLineCenterIndex.append(corner_structure_list.orig_coordinates.at(i).z);
                remarkable_corners.near_horizontal.push_back(cleaned_corners.at(i));
                circle( clean_scaled_src, Point(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z), 5,  CV_RGB(255,0,0), 2, 8, 0 );
                break;
            }
        }
    }

    // Vertical
    for( int i = 0; i < corner_structure_list.orig_coordinates.size(); i++ )
    {
        for (int l=0; l<scaled_chess.vertical_pointlike.size();l++)
        {
            if(cv::norm(scaled_chess.vertical_pointlike.at(l) - Point2f(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z)) <= 40 &&
                    corner_structure_list.orig_coordinates.at(i).z - scaled_chess.vertical_pointlike.at(l).y > 0)
            {
                remarkable_corners.VerLineCenterIndex.append(corner_structure_list.orig_coordinates.at(i).x);
                remarkable_corners.DX_vertical.append(corner_structure_list.orig_coordinates.at(i));
                circle( clean_scaled_src, Point(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z), 5,  CV_RGB(255,0,0), 2, 8, 0 );
                break;
            }

            if(cv::norm(scaled_chess.vertical_pointlike.at(l) - Point2f(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z)) <= 50 &&
                    corner_structure_list.orig_coordinates.at(i).z - scaled_chess.vertical_pointlike.at(l).y < 0)
            {
                remarkable_corners.VerLineCenterIndex.append(corner_structure_list.orig_coordinates.at(i).x);
                remarkable_corners.SX_vertical.append(corner_structure_list.orig_coordinates.at(i));
                circle( clean_scaled_src, Point(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z), 5,  CV_RGB(255,0,0), 2, 8, 0 );
                break;
            }
        }
    }

    imshow("bla bla", clean_scaled_src);
    waitKey(0);



    /// Y-SORTING
    std::sort(corner_structure_list.delta_corrected_sorted_coordinates.begin(), corner_structure_list.delta_corrected_sorted_coordinates.end(), POINTS3_COMPARE_Z);

    /*
            // DA ESEGUIRE RIGA PER RIGA
        // Last sanity check. If corners are not equidistant, erase it.

        for (int u=1; u< corner_structure_list.delta_corrected_sorted_coordinates.size() - 1 ; u++)
        {
            int r_dist = fabs(corner_structure_list.delta_corrected_sorted_coordinates.at(u).y -
                               corner_structure_list.delta_corrected_sorted_coordinates.at(u+1).y);
            int l_dist= fabs(corner_structure_list.delta_corrected_sorted_coordinates.at(u).y -
                             corner_structure_list.delta_corrected_sorted_coordinates.at(u-1).y);
            int r_elev_dist= fabs(corner_structure_list.delta_corrected_sorted_coordinates.at(u).z -
                               corner_structure_list.delta_corrected_sorted_coordinates.at(u+1).z);
            int l_elev_dist= fabs(corner_structure_list.delta_corrected_sorted_coordinates.at(u).z -
                               corner_structure_list.delta_corrected_sorted_coordinates.at(u-1).z);

            if (r_elev_dist < 3 && l_elev_dist < 3 && fabs(r_dist - l_dist) > 10)
            {
                corner_structure_list.delta_corrected_sorted_coordinates.remove(u);
            }
        }


*/


    // Create a idx-sorted version
    QVector <Point3f> auxil;

    auxil=corner_structure_list.delta_corrected_sorted_coordinates;
    std::sort(auxil.begin(), auxil.end(), POINTS3_COMPARE_X);
    corner_structure_list.delta_corrected_idx_sorted=auxil;



    /*

                            /////////////

                            //Horizontal lines Z calib


                            QList < calib_z_3Dpoint_t > CornerZ3DList; //List of corners with knowledge of Y line number
                            int lineZindex = 1;

                            // We know how distant I am from the initial line in term of mm

                            for(int i = 0; i< cleaned_corners.size(); i++)
                            {
                                calib_z_3Dpoint_t CornerZ3D;
                                CornerZ3D.CornerPoint = cleaned_corners.at(i);
                                CornerZ3D.zLineMMHeight = (lineZindex)*square_size_mm;
                                CornerZ3DList.append(CornerZ3D);

                                if( ((i+1) < cleaned_corners.size()) &&  fabs((int)(cleaned_corners.at(i).x - cleaned_corners.at(i+1).x)) > (ZthrXDiff_pix)) ////////

                                {
                                    lineZindex++;
                                }
                                else continue;
                            }

                        */





    QList < calib_z_3Dpoint_t > CornerZ3DList; //List of corners with knowledge of Y line number
    int lineZindex = 1;
    QVector < QVector <Point3f> > lines_idx;
    QVector <Point3f> aux;
    QList<  line_Z_corner_t  > LineZCornerList;
    line_Z_corner_t lineZcorner;
    int counter=1;

    ///// ADDING A FAKE CORNER AT THE END OF THE VECTOR TO EASE THE SORTING OF THE LAST ROW
    ///
    ///
    ///

    //corner_structure_list.delta_corrected_sorted_coordinates.append(Point3f(0,0,2500));

    for(int i = 0; i< corner_structure_list.delta_corrected_sorted_coordinates.size(); i++)
    {
        calib_z_3Dpoint_t CornerZ3D;





        CornerZ3D.CornerPoint= retrieve_orig_coord_from_idx(corner_structure_list, corner_structure_list.delta_corrected_sorted_coordinates.at(i).x);

        CornerZ3D.CornerIndex=corner_structure_list.delta_corrected_sorted_coordinates.at(i).x;

        if (corner_structure_list.delta_corrected_sorted_coordinates.at(i).z > scaled_chess.hor_line_quote)
        {
            CornerZ3D.zLineMMHeight = (lineZindex +1) *square_size_mm;
        }

        else
        {
            CornerZ3D.zLineMMHeight = (lineZindex)*square_size_mm;

        }

        CornerZ3DList.append(CornerZ3D);

        aux.push_back(corner_structure_list.delta_corrected_sorted_coordinates.at(i));

        if( ((i+1) < corner_structure_list.delta_corrected_sorted_coordinates.size()) &&
                fabs((corner_structure_list.delta_corrected_sorted_coordinates.at(i).z -
                      corner_structure_list.delta_corrected_sorted_coordinates.at(i+1).z)) > (15)

                || (i+1) == (int)corner_structure_list.delta_corrected_sorted_coordinates.size() )

        {
            lineZindex++;
            lineZcorner.CornerNumber= counter;

            std::sort(aux.begin(), aux.end(), POINTS3_COMPARE_Y);

            for (int m=0; m< aux.size(); m++)
            {
                for (int g=0; g < remarkable_corners.DX_vertical.size(); g++)
                {
                    if (aux.at(m).x == remarkable_corners.DX_vertical.at(g).x)
                    {
                        lineZcorner.CenterIndex= remarkable_corners.DX_vertical.at(g).x;
                        LineZCornerList.append(lineZcorner);
                        break;
                    }
                    else continue;
                }
            }
        }

        else
        {
            counter++;

            continue;
        }

        lines_idx.append(aux);
        aux.clear();
        counter=1;
    }








    // x-sorting each row using indexes


    for (int a=0; a < lines_idx.size(); a++)
    {
        QVector <Point3f> aiut;
        aiut=lines_idx.at(a);
        std::sort(aiut.begin(), aiut.end(), POINTS3_COMPARE_Y);

        for (int q=0; q < aiut.size(); q++)
        {
            corner_structure_list.rows_x_sorted.push_back(aiut.at(q));
        }
        aiut.clear();
    }



    //Last proximity filter


    for (int a=0; a < lines_idx.size(); a++)
    {   QVector <int> auxilia;
        for (int u=0; u< lines_idx.at(a).size() -1; u++)
        {
            auxilia.push_back(fabs(lines_idx.at(a).at(u).y - lines_idx.at(a).at(u+1).y));
        }
        int m= (std::accumulate(auxilia.begin(), auxilia.end(), 0))/(auxilia.size());

        QVector <Point3f> aiut;
        aiut=lines_idx.at(a);
        std::sort(aiut.begin(), aiut.end(), POINTS3_COMPARE_Y);

        for (int l=1; l< lines_idx.at(a).size() -1; l++)
        {
            int dr=fabs(aiut.at(l).y - aiut.at(l+1).y);
            int dl=fabs(aiut.at(l).y - aiut.at(l-1).y);

            if (fabs(dr - m) > 10 &&  fabs(dl - m) > 10)
            {
                cout << aiut.at(l) << endl;
                l+=2;
                circle(clean_scaled_src, Point(aiut.at(l).y, aiut.at(l).z), 10, CV_RGB(255,0,0), 2, 8, 0);
            }
            else continue;
        }
        auxilia.clear();
    }

    imshow("INILUSNS", clean_scaled_src);
    waitKey(0);


    /*

 ////////////////////////////////////////////////////////////////////////////////////////////////////////

        //Assigning corners' indexes to lines


        QVector < QVector <int> > lines_idx;
        int lineZindex = 1;

        for(int i = 0; i< corner_structure_list.delta_corrected_sorted_coordinates.size(); i++)
        {


            if( ((i+1) < corner_structure_list.delta_corrected_sorted_coordinates.size()) &&
                    fabs((int)(corner_structure_list.delta_corrected_sorted_coordinates.at(i).z -
                               corner_structure_list.delta_corrected_sorted_coordinates.at(i+1).z)) > (15))

            {   lines_idx.at(lineZindex).append((int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).x);
                lineZindex++;
            }
            else continue;
        }





*/






    // Hic sunt leones
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    // Y calib

    // Find center: central black line; on the chessboard there will be a reference central black band

    // QList<  line_Z_corner_t  > LineZCornerList;
    //  line_Z_corner_t lineZcorner;
    // LineZCornerList.clear();


    /// FOR Y CALIBRATION
    /// find single line of all black squares VERTICAL
    ///


    /*

      //int countBlack;
        //int countWhite;

       // int SumBlackVert =0;
        //QVector< int > vecBlackVert;
        for( int i = LimitInVerBlackLine; i < refined.cols-LimitInVerBlackLine; i++ )
        {
            countWhite =0;
            countBlack =0;
            for( int j = 0; j < refined.rows ; j++ )
            {
                if((int)refined.at<uchar>(j,i) < mean_refined)
                {
                    countBlack++;
                }
                else
                {
                    countWhite++;
                }
            }
            if(countBlack > PercBlack*countWhite)
            {
                vecBlackVert.append(i);
            }
        }
        // If I have more than 0 lines in which number of pixels black are > 3*white pixels; I consider the mean line

        if(vecBlackVert.size() >0)
        {
            for( int i = 0; i < vecBlackVert.size(); i++ )
            {
                SumBlackVert+= vecBlackVert.at(i);
            }
            SumBlackVert = SumBlackVert/vecBlackVert.size();
        }



    // I select the corners at the right to the central all black line

    for( int i = 0; i < cleaned_corners.size(); i++ )
    {
        //I calculate the corner distance from the image center

        if((abs(cleaned_corners[i].x-( ImgWidth/2)))< MinDistFromCenter)
        {
            MinDistFromCenter=(abs(cleaned_corners[i].x-(ImgWidth/2)));
            index = i;
        }


        if(((i+1) < cleaned_corners.size()) && ((int)cleaned_corners[i+1].x - (int)cleaned_corners[i].x)>(-ZthrXDiff_pix))
        {
            if((vecBlackVert.size() >0) && ((((int)cleaned_corners.at(i).x - SumBlackVert)) <= VerProxCenterBlackLine && (((int)cleaned_corners.at(i).x - SumBlackVert)) >0 ))
            {
                CenterFound = true;
                lineZcorner.CenterIndex = (i);


                circle( img_FINAL, Point(cleaned_corners[i].x,cleaned_corners[i].y), 3,  CV_RGB(255,0,0), 2, 8, 0 );

                MinDistFromCenter = 1000;
                index = 0;
            }
            CountCorn++;
        }
        else
        {
            if(CenterFound == false)
            {
                lineZcorner.CenterIndex = index;

                circle( img_FINAL, Point(cleaned_corners[lineZcorner.CenterIndex].x,cleaned_corners[lineZcorner.CenterIndex].y), 3,  CV_RGB(0,0,255), 2, 8, 0 );
            }
            lineZcorner.CornerNumber = (CountCorn);
            LineZCornerList.append(lineZcorner);

            CountCorn =1;
            CenterFound = false;
            MinDistFromCenter = 1000;
            index = 0;
        }
    }





    */



    /*
    int MinDistFromCenter = ImgWidth;
    int index;
    int CountCorn = 1;
    bool CenterFound = false;


    // I select the corners at the right to the central all black line

    for( int i = 0; i < corner_structure_list.delta_corrected_sorted_coordinates.size(); i++ )
    {
        //I calculate the corner distance from the image center
        int quote_projection= scaled_chess.vertical_pointlike.at(corner_structure_list.delta_corrected_sorted_coordinates.at(i).z).x;

        if((abs( corner_structure_list.delta_corrected_sorted_coordinates.at(i).y - 950 )) < MinDistFromCenter)
        {
            MinDistFromCenter=abs(corner_structure_list.delta_corrected_sorted_coordinates.at(i).y - 950 );
            index = i;
        }

        if(((i+1) < corner_structure_list.delta_corrected_sorted_coordinates.size()) &&
                ((int)corner_structure_list.delta_corrected_sorted_coordinates.at(i+1).y -
                 (int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).y)> - 950) //
        {

            if(
                    ((int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).y - scaled_chess.ver_line_quote) <= 40 &&
                    ((int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).y - scaled_chess.ver_line_quote) >0 )
            {
                CenterFound = true;
                lineZcorner.CenterIndex = (corner_structure_list.delta_corrected_sorted_coordinates.at(i).x);

                circle( clean_scaled_src,Point(corner_structure_list.delta_corrected_sorted_coordinates.at(i).y,corner_structure_list.delta_corrected_sorted_coordinates.at(i).z), 10,  CV_RGB(255,0,0), 2, 8, 0 );


                MinDistFromCenter = ImgWidth;
                index = 0;
            }

            CountCorn++;

        }

        else

        {
            if(CenterFound == false)
            {
                lineZcorner.CenterIndex = corner_structure_list.delta_corrected_sorted_coordinates.at(i).x;
                circle( clean_scaled_src, Point(corner_structure_list.orig_coordinates.at(lineZcorner.CenterIndex).y,corner_structure_list.orig_coordinates.at(lineZcorner.CenterIndex).z), 3,  CV_RGB(0,0,255), 2, 8, 0 );

            }

            lineZcorner.CornerNumber = (CountCorn);
            LineZCornerList.append(lineZcorner);

            CountCorn =1;
            CenterFound = false;
            MinDistFromCenter = ImgWidth;
            index = 0;
        }
    }

    imshow("bla bla", clean_scaled_src);
    waitKey(0);





*/




    // E' QUA QUANDO LA CERCHI




    /*
     *
    // We identify the vertical lines, starting from the reference line

    QList < calib_y_3Dpoint_t > CornerY3DList;//List of corners with knowledge of Y line number
    int lineZcount = 0;


    // We  know how distant I am from center in term of mm


    for(int i = 0; i<cleaned_corners.size(); i++)
    {
        calib_y_3Dpoint_t CornerY3D;
        CornerY3D.CornerPoint = cleaned_corners[i];


        CornerY3D.yLineMMdistance = (((int) i -  LineZCornerList.at(lineZcount).CenterIndex)*square_size_mm);


        CornerY3DList.append(CornerY3D);

        if(((i+1) < cleaned_corners.size()) && ((int)cleaned_corners[i+1].x- (int)cleaned_corners[i].x)<(-ZthrXDiff_pix))
        {
            lineZcount++;
        }
    }
    //////////////

    */





    // Y-spatial

    //Medium z-size of squares sorted by y-quote.

    QVector <Point2f> squares_y_dim_px; // (mean_quote, square_longest_dim)
    std::sort(remarkable_corners.DX_vertical.begin(), remarkable_corners.DX_vertical.end(), POINTS3_COMPARE_Z);
    std::sort(remarkable_corners.SX_vertical.begin(), remarkable_corners.SX_vertical.end(), POINTS3_COMPARE_Z);

    for (int a=0; a <remarkable_corners.SX_vertical.size(); a++)

    {

        if (remarkable_corners.DX_vertical.at(a).z - remarkable_corners.SX_vertical.at(a).z < 10)
        {
            int mean_quote= floor(.5F*(remarkable_corners.DX_vertical.at(a).z + remarkable_corners.SX_vertical.at(a).z));
            int square_maj_dim= fabs((int) remarkable_corners.DX_vertical.at(a).y - remarkable_corners.SX_vertical.at(a).y);
            squares_y_dim_px.push_back(Point2f(mean_quote, square_maj_dim));

        }
        else continue;
    }


    /*

    // We identify the vertical lines, starting from the reference line

    QList < calib_y_3Dpoint_t > CornerY3DList;//List of corners with knowledge of Y line number
    int lineZcount = 0;


    // We  know how distant I am from center in term of mm


    for(int i = 0; i<corner_structure_list.delta_corrected_sorted_coordinates.size(); i++)
    {
        calib_y_3Dpoint_t CornerY3D;

        CornerY3D.CornerPoint = Point2f(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z);


        //CornerY3D.yLineMMdistance = (((int) i -  LineZCornerList.at(lineZcount).CenterIndex)*square_size_mm);


        CornerY3DList.append(CornerY3D);

        if(((i+1) < corner_structure_list.delta_corrected_sorted_coordinates.size()) &&

                ((int)corner_structure_list.delta_corrected_sorted_coordinates.at(i+1).x -

                 (int)corner_structure_list.delta_corrected_sorted_coordinates.at(i).x > 1000))
        {
            lineZcount++;
        }

    }
    //////////////




*/



    // We identify the vertical lines, starting from the reference line

    QList < calib_y_3Dpoint_t > CornerY3DList;//List of corners with knowledge of Y line number
    int lineZcount = 1;


    // We  know how distant I am from center in term of mm

    int square_size_px=0;
    for(int i = 0; i<corner_structure_list.rows_x_sorted.size(); i++)
    {
        calib_y_3Dpoint_t CornerY3D;

        int id=corner_structure_list.rows_x_sorted.at(i).x;

        CornerY3D.CornerPoint=retrieve_orig_coord_from_idx(corner_structure_list, id);

        //CornerY3D.CornerPoint = Point2f(corner_structure_list.orig_coordinates.at(i).y, corner_structure_list.orig_coordinates.at(i).z);
        CornerY3D.CornerIndex= corner_structure_list.orig_coordinates.at(i).x;

        // Select the correct square_distance;

        for (int n=0; n< squares_y_dim_px.size(); n++)
        {
            if (fabs((corner_structure_list.rows_x_sorted.at(i).z - squares_y_dim_px.at(n).x <10 )))
            {
                square_size_px+=squares_y_dim_px.at(n).y;
                break;
            }
            else continue;
        }

        //Distanza in pixels dalla verticale

        float px_distance_from_vertical= corner_structure_list.rows_x_sorted.at(i).y -
                scaled_chess.vertical_pointlike.at(corner_structure_list.rows_x_sorted.at(i).z).x;

        //Distanza in quadrati dalla verticale
        int squares_dist_from_vertical= (((double) px_distance_from_vertical)/((double)square_size_px));

        CornerY3D.yLineMMdistance= squares_dist_from_vertical* square_size_mm;


        //CornerY3D.yLineMMdistance = (((int) i -  LineZCornerList.at(lineZcount).CenterIndex)*square_size_mm);


        CornerY3DList.append(CornerY3D);

        if(((i+1) < corner_structure_list.rows_x_sorted.size()) &&

                ((int)corner_structure_list.rows_x_sorted.at(i+1).y -

                 (int)corner_structure_list.rows_x_sorted.at(i).y < - 980) )
        {
            lineZcount++;
        }
        square_size_px=0;
    }
    //////////////






    ///////////////////////////////////////////////////////////////////





























    /// Showing the result



    img_grayscale.release();
    img_color.release();

    //Interpolation -> I create the Y and Z calib tables (matrix)
    int t = 0;
    int TotalCornerNumber =0;
    for(int i = 0; i<LineZCornerList.size(); i++) //for on how many z lines I have
    {
        calib_y_Line_t YLine;
        calib_z_Line_t ZLine;

        TotalCornerNumber+=LineZCornerList.at(i).CornerNumber;


        for(unsigned int j = 0; j<ImgWidth; j++) //for on Image Width in pixels
        {

            //We ask for the z value (in pixels) i.e. the height of the central point of each zline

            // YLine.ZlineCoord.append( (int)CornerY3DList.at(LineZCornerList.at(i).CenterIndex).CornerPoint.y); ORIGINALE
            YLine.ZlineCoord.append(squares_y_dim_px.at(i).x );

            // The distance from the central reference of each pixel of linez = Ydistance

            YLine.unlinY.append(interpolate(CornerY3DList.at(t).CornerPoint.x,CornerY3DList.at(t).yLineMMdistance,
                                            CornerY3DList.at(t+1).CornerPoint.x, CornerY3DList.at(t+1).yLineMMdistance,j));


            //We ask for the z value (in mm) i.e. the height of the central point of each zline

            ZLine.ZlineHeight.append( (int)CornerZ3DList.at(LineZCornerList.at(i).CenterIndex).zLineMMHeight);


            // The distance from the initial line

            ZLine.unlinZ.append(interpolate(CornerZ3DList.at(t).CornerPoint.x,CornerZ3DList.at(t).CornerPoint.y, CornerZ3DList.at(t+1).CornerPoint.x, CornerZ3DList.at(t+1).CornerPoint.y,j));

            if(t < TotalCornerNumber-2 && j > CornerY3DList.at(t+1).CornerPoint.x )
            {
                t++;
            }
        }
        tableY.append(YLine);
        tableZ.append(ZLine);
        t=t+2;
    }
    ///////////////////////

    // Save tables

    QString pathZ, pathY;

    pathZ = output_dir + "table_z.smpacq";
    pathY = output_dir + "table_y.smpacq";

    // Z table
    if ( !pathZ.isEmpty() )
    {
        QFile table_out(pathZ);
        if ( !table_out.open( QIODevice::Text | QIODevice::WriteOnly ) )
        {
            qDebug()<<"Error opening Z";
        }
        else
        {
            char buffer[256];
            for ( unsigned int k = 0; k < ImgWidth; k++ )
            {
                for ( int i = 0; i < tableZ.size(); ++i )
                {
                    if ( tableZ.at(i).unlinZ.at(k) != 0 )
                    {
                        snprintf( buffer, 256, "%f %f ", tableZ.at(i).unlinZ.at(k), tableZ.at(i).ZlineHeight.at(k) );
                        table_out.write( buffer );
                    }
                }
                table_out.write("*\n");
            }
        }
        table_out.flush();
        table_out.close();
    }
    // Y table
    if ( !pathY.isEmpty() )
    {
        QFile table_out(pathY);
        if ( !table_out.open( QIODevice::Text | QIODevice::WriteOnly ) )
        {
            qDebug()<<"Error opening Y";
        }
        else
        {
            char buffer[256];
            for ( unsigned int k = 0; k < ImgWidth; k++ )
            {

                for ( int i = 0; i < tableY.size(); ++i )
                {
                    snprintf( buffer, 256, "%d %f ", tableY.at(i).ZlineCoord.at(k), tableY.at(i).unlinY.at(k) );
                    table_out.write( buffer );
                }
                table_out.write("*\n");
            }
        }
        table_out.flush();
        table_out.close();
    }

    ///////////////

    // Clear Lists and Vectors
    tableZ.clear();
    tableY.clear();
    LineZCornerList.clear();
    CornerY3DList.clear();
    CornerZ3DList.clear();
    corners.clear();
    addedIndex.clear();

}
