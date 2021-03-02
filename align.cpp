#include <opencv2/core/core.hpp>
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
#include <align.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <QElapsedTimer>
#include <numeric>
#include <functional>
#include <iterator>
#include <stdexcept>



using namespace std;
using namespace cv;


/// All the quote are measured with respect to the (0,0) origin point of the image thatis assumed to be the top-left corner. So the following assumptions are made:
/// y-coordinate (rows of the image) are growing top-bottom;
/// x-coordinate (cols of the image) are growing top-bottom;



// Function that extract the medium quote and the point-like representation of the median of the horizontal black strip present on the target chessboard.
int extract_horizontal_black_line (chessboard_properties &chess, sources &src)
{
    std::vector <int> mean_black;
    vector <int> row_val;
    QVector <Point2f> aux;
    int top_offset=50;
    int bottom_offset=20;


    for (int r= chess.mean_top_limit + top_offset; r< chess.mean_bottom_limit - bottom_offset; r++) ///

    {   row_val.clear();
        aux.clear();
        QVector <Point2f> aux;
        for (int c=0; c < src.bin.cols ; c++)
        {
            row_val.push_back(src.bin.at<uchar>(r - chess.deltas.at(c).y,c));
            continue;
        }

         if (row_val.size() > 0)
        {
            if (floor(std::accumulate(row_val.begin(), row_val.end(), 0)/(row_val.size())) < 10 ) // Max 50 px white to assume the line black. Due to noise on the top white line
            {
                mean_black.push_back(r);

            }
            else continue;
        }
    }

    if( mean_black.size()> 10)
    {
        int median_black_row_idx = ceil(accumulate(mean_black.begin(), mean_black.end(), 0)/(mean_black.size()));

        chess.crosshair.horizontal=true;

        QVector <float> gaps;

        for (int n =0; n < src.grayscale.cols; n++)
        {
            chess.horizontal_pointlike.push_back(Point2f(n, median_black_row_idx - chess.deltas.at(n).y));
            gaps.push_back(chess.deltas.at(n).y);
        }

        // Quote of the horizontal will be medium row index of the black strip minus the medium gap between the left and right extreme
        chess.hor_line_quote = median_black_row_idx - floor(accumulate(gaps.begin(), gaps.end(), 0) /gaps.size() );

    }

    else
    {
        chess.crosshair.horizontal=false;
        cout << "Left image is very tilted, please consider checking the calibration tool's setup. We'll still attempt to calibrate." << endl;
    }


    // Approx mode.

    if (chess.crosshair.horizontal==false)

    {
        cout << "Approx_mode" << endl;
        vector <Point2i> black_amount_per_rows; //x is the x (col) of the point, y the black amount;

        for (int r= chess.mean_top_limit; r< chess.mean_bottom_limit; r++)
        {int b_amount=0;
            aux.clear();
            for (int c= 0; c <src.bin.cols; c++)
            {
                if (src.bin.at<uchar>(r - chess.deltas.at(c).y,c) < 2)
                {
                    b_amount+=1;
                    continue;
                    aux.push_back(Point2f(c,r));
                }
            }

            black_amount_per_rows.push_back(Point2i(r, b_amount));
        }

        points_sort_y compare_height;

        std::sort(black_amount_per_rows.begin(), black_amount_per_rows.end(), compare_height);

        vector <int > v;

        for (int i=0; i < black_amount_per_rows.size(); i++)
        {// Salvo un valore di nero se differisce dal massimo numero di neri (centro della distribuzione) per meno di 5

            if (fabs(black_amount_per_rows.at(i).y - black_amount_per_rows.at((int) black_amount_per_rows.size() -1).y) <5)
            {
                v.push_back(black_amount_per_rows.at(i).x);
                continue;
            }
        }
        chess.hor_line_quote=floor(accumulate(v.begin(), v.end(), 0)/(v.size()));
        chess.horizontal_pointlike=aux;
        black_amount_per_rows.clear();
        v.clear();
    }
    row_val.clear();
    mean_black.clear();

    return 0;

}


// Function that extract the medium quote and the point-like representation of the median of the vertical black strip present on the target chessboard.

int extract_vertical_black_line  (chessboard_properties &chess, sources &src)
{


    vector <int> col_val;
    std::vector<int> mean_black;

    for (int col=0; col < src.grayscale.cols; col++)

    {col_val.clear();
        for (int row= (int) chess.mean_top_limit; row< (int) chess.mean_bottom_limit; row++) ///

        {
            col_val.push_back(src.grayscale.at<uchar>(row,col));
        }

        if (col_val.size() > 0)
        {
            if (floor(std::accumulate(col_val.begin(), col_val.end(), 0)/(col_val.size())) < 10 ) // Max 50 px white to assume the line black. Due to noise on the top white line
            {
                mean_black.push_back(col);

            }
            else continue;

        }

    }

    if( mean_black.size() > 10)
    {
        chess.ver_line_quote=floor(accumulate(mean_black.begin(), mean_black.end(), 0)/(mean_black.size()));
        chess.crosshair.vertical=true;
    }

    else {
        chess.crosshair.vertical=false;
    }


    // Approx mode

    if (chess.crosshair.vertical==false)
    {
        vector <Point2i> black_amount_per_columns; //x is the x (col) of the point, y the black amount;

        for (int c=0; c< src.bin.cols; c++)
        {int b_amount=0;

            for (int r= (int) chess.mean_top_limit; r< (int) chess.mean_bottom_limit; r++)
            {
                if (src.bin.at<uchar>(r,c) < 2)
                {
                    b_amount+=1;

                    continue;

                }
            }
            black_amount_per_columns.push_back(Point2i(c, b_amount));
        }

        points_sort_y compare_height;

        std::sort(black_amount_per_columns.begin(), black_amount_per_columns.end(), compare_height);

        vector <int > v;

        for (int i=0; i < black_amount_per_columns.size(); i++)
        {
            if (fabs(black_amount_per_columns.at(i).y - black_amount_per_columns.at((int) black_amount_per_columns.size() -1).y) <5)
            {
                v.push_back(black_amount_per_columns.at(i).x);
                continue;
            }
        }
        chess.ver_line_quote=floor(accumulate(v.begin(), v.end(), 0)/(v.size()));


        black_amount_per_columns.clear();
        v.clear();

    }

    col_val.clear();
    mean_black.clear();
    return 0;

}



// Function that extract the median quote of the upper and lower border of the target chessboard.
int extract_chess_limits (chessboard_properties &chess, sources &src, bool above, bool below)
{

    vector  <Point2i> strongholds;

    for (int i=2; i<=100; i++)
    {
        Point2i foo(0, i *(src.grayscale.cols/100) );
        strongholds.push_back(foo);

    }
    if (above)
    {
        // FROM ABOVE

        threshold(src.grayscale, src.bin, 50, 255, THRESH_BINARY);

        vector <int> up_quotes;

        for (int n=10; n< strongholds.size(); n++)

        { int quote =0;

            for (int m=0; m <= src.grayscale.rows ; m++)
            {
                if (src.bin.at<uchar>(m, strongholds.at(n).y) < 200)
                {
                    quote+=1;
                }
                else break;
            }
            if(quote < src.grayscale.rows -100) ////MIGLIORARE
            {
                up_quotes.push_back(quote);
            }
            else continue;
        }

        int avg_top_quotes=0;

        avg_top_quotes=floor(accumulate(up_quotes.begin(), up_quotes.end(), 0)/(up_quotes.size()));
        chess.mean_top_limit = (0+ avg_top_quotes);
        up_quotes.clear();
    }


    // FROM BELOW
    if (below)

    {

        vector <int> down_quotes;
        for (int n=1; n< strongholds.size(); n++)
        { int quote =0;


            for (int m=src.grayscale.rows; m > 0  ; --m)
            {
                if (src.bin.at<uchar>(m, strongholds.at(n).y) < 200)
                {
                    quote+=1;
                }
                else break;
            }
            if (quote < src.grayscale.rows - 100) //Sanity check
            {
                down_quotes.push_back(quote);
            }
            else continue;
        }

        int avg_bottom_quotes=0;

        avg_bottom_quotes=floor(accumulate(down_quotes.begin(), down_quotes.end(), 0)/(down_quotes.size()));
        chess.mean_bottom_limit= src.grayscale.rows - avg_bottom_quotes;
        down_quotes.clear();



    }
    else
    {chess.mean_bottom_limit=src.grayscale.rows -10;}



    return 0;


}


// Function that extract the x_correction

int extract_x_correction(chessboard_properties &chess,sources &src)
{


    QVector <Point2f>  absolute_vertical;
    for (int z=0; z< src.grayscale.rows; z++)
    {
        absolute_vertical.push_back(Point2f(chess.ver_line_quote ,z ));
    }



    for (int j= chess.ver_line_quote - chess.hor_line_quote;
         j< (chess.ver_line_quote + (src.grayscale.rows - chess.hor_line_quote)); j++)
    {
           chess.vertical_pointlike.push_back(Point2f(chess.ver_line_quote - chess.x_center_deltas.at(j).y, fabs(chess.ver_line_quote - chess.hor_line_quote - j )));
    }


    for (int a=0; a < absolute_vertical.size(); a++)
    {//(Coordinate x (row) of the point, delta(x) with respe)

        chess.y_center_deltas.push_back(Point2i(a, chess.vertical_pointlike.at(a).x - chess.vertical_pointlike.at(.5F*src.grayscale.rows).x));
    }
}

// Function that extract the inclined horizontal border of the image and it's deviation point-like-wise from the absolute horizontal.
// We assume the sin(x)~x first order developement that is an acceptable approximation for angles ≤ 7°.
int extract_relative_dimensions (chessboard_properties &chess, sources &src, int col_start, int col_final )
{

    vector <Point2i> deltas;
    QVector <Point2i> relative_horizontal;

    for (int c=col_start; c< col_final; c++)
    {
        for (int r=0; r < src.bin.rows; r++)
        {
            if (src.bin.at<uchar>(r,c)>200)
            {
                // Correction quotes will be calculated taking the y of the point at the far left of the line.

                relative_horizontal.push_back(Point2i(c,r));
                deltas.push_back(Point2i(c, (relative_horizontal.at(0).y - r)));
                break;
            }
        }
    }




    for (int k =0; k < relative_horizontal.size(); k++)
    {//(Coordinate y (column) of the point, delta(y))
    chess.x_center_deltas.push_back(Point2i(k, (relative_horizontal.at(k).y - relative_horizontal.at(0.5F*src.grayscale.cols).y)));

    }




    for (int p=0; p< relative_horizontal.size(); p++)
    {
        chess.top_contour.push_back(relative_horizontal.at(p));
        chess.deltas.push_back(deltas.at(p));
    }
    relative_horizontal.clear();
    deltas.clear();





    return 0;
}



// Auxiliary function that return true if the medium color of a row is < 10 i.e. can be assumed to be full black.
bool sanity_check(sources &src, int needed_border)
{
    QVector <float> aux;

    for (int c=0; c < src.bin.cols; c++)
    {
        for (int r=0; r<=needed_border; r++)
        {
            aux.push_back(src.bin.at<uchar>(r,c));
        }
    }

    if(floor(accumulate(aux.begin(), aux.end(), 0)/aux.size()) < 10)
    {
        return true;
    }

    else
    {
        return false;
    }

}



// Function that operate the cut of raw image to 1000x300.
void adaptive_cut(chessboard_properties &chess, sources &src, sources &src_cropped, int top_black_border)
{
    sources try_crop = src;

    for (int up_cut=130; up_cut < 180; up_cut++)
    {
        try
        {
            chess.corner_up_left.x = (chess.ver_line_quote - 500);
            chess.corner_up_left.y= (chess.hor_line_quote-up_cut);
            chess.corner_down_right.x = chess.ver_line_quote + 500;
            chess.corner_down_right.y = chess.hor_line_quote +(300 - up_cut);
            chess.calibration_point= Point2i(chess.ver_line_quote -500, chess.hor_line_quote - up_cut);

            if (chess.hor_line_quote-up_cut < 0)
            {
                throw "Exceeded image top limit";
            }
        }
        catch (const std::out_of_range &oor)
        {
            std::cerr << "Out of Range error: " << oor.what() << '\n';
            continue;
        }

        cv::Rect alt_right_crop_roi(chess.corner_up_left.x,(int) (chess.corner_up_left.y), 1000, 300);

        try_crop.grayscale = src.grayscale(alt_right_crop_roi);
        try_crop.color=src.color(alt_right_crop_roi);
        threshold(try_crop.grayscale, try_crop.bin, 70, 255, THRESH_BINARY);

        if (sanity_check(try_crop, top_black_border)!=true)
        {
            continue;
        }

        else
        {
            src_cropped.grayscale= try_crop.grayscale;
            src_cropped.bin= try_crop.bin;
            src_cropped.color= try_crop.color;
            break;
        }
    }

    try_crop.grayscale.release();
    try_crop.bin.release();
    try_crop.color.release();
    return ;
}



// Function that create a vector tilted with the deltas extracted with extract_relative_horizontal function starting for a quote (row "int R").
void create_transformed_vec2f(int R, QVector <Point2f> &vec_dst, chessboard_properties &chess_properties, sources &img_src )
{
    for (int c=0; c< img_src.grayscale.cols; c++)
    {
        vec_dst.push_back(Point2f(c, R - chess_properties.deltas.at(c).y));
    }
    return ;

}




// Function that align, crop, enhance and write the image that will be used by the calibration routine.

void align(char* imgs_filename , chessboard_properties &left_chess , chessboard_properties &right_chess ,chessboard_properties &left_chess_cropped, chessboard_properties &right_chess_cropped )
{

    /// Source images loaded both in grayscale than in original colors. The grayscale one is the copy
    /// we'll be working on, color one will be used to show results on the GUI.

    sources left_src, right_src; // 1280x800
    float smooth= .00001;

    right_src.color= imread("/home/ale/Desktop/acquisizioni_v2_0709/tilted_artificiali/right_tilted.bmp", IMREAD_COLOR);
    right_src.grayscale= imread("/home/ale/Desktop/acquisizioni_v2_0709/tilted_artificiali/right_tilted.bmp", IMREAD_GRAYSCALE);


    left_src.grayscale= imread("/home/ale/Desktop/acquisizioni_v2_0709/tilted_artificiali/left_tilted.bmp" , IMREAD_GRAYSCALE);
    left_src.color=imread("/home/ale/Desktop/acquisizioni_v2_0709/tilted_artificiali/left_tilted.bmp", IMREAD_COLOR);
    // imshow("right", right.grayscale);


    // LOW THRESHOLDED SOURCES

    threshold(right_src.grayscale, right_src.bin, 70, 255, THRESH_BINARY);
    threshold(left_src.grayscale, left_src.bin, 70, 255, THRESH_BINARY);


    imshow("left", left_src.bin);
    waitKey(0);

    imshow("right", right_src.bin);
    waitKey(0);


    destroyAllWindows();


    // DIMENSIONAL CONTROLS

    if (left_src.grayscale.cols < 1280 ||right_src.grayscale.cols < 1280)
    {
        cout << "Please control the setup of the imager: width of the raw image is < 1280" << endl;
    }

    if (left_src.grayscale.cols > 1280 || right_src.grayscale.cols > 1280)
    {
        cout << "Please control the setup of the imager: width of the raw image is > 1280" << endl;
    }

    if (left_src.grayscale.rows <800 || right_src.grayscale.rows < 800 )
    {
        cout << "Please control the setup of the imager: height of the raw image is < 800" << endl;
    }

    if (left_src.grayscale.rows >800 || right_src.grayscale.rows > 800 )
    {
        cout << "Please control the setup of the imager: height of the raw image is > 800" << endl;
    }


    // FINDING THE CHESSBOARD on the raw image

    extract_chess_limits(left_chess,left_src, true, true);
    extract_chess_limits(right_chess, right_src, true, true);


    /// Instantiate the chessboard structures:
    ///
    /// int mean_top_limit: height of the black region uppermost the chessboard with respect to the raw image (1200x800);
    /// int mean_bottom_limit: height of the black region beneath the chessboard with respect to the raw image (1200x800);
    /// int hor_line_quote: mean quote of the horizontal black line with respect to the raw image (1200x800)
    /// int ver_line_quote: mean quote of the vertical black line with respect to the raw image (1200x800)
    /// Point2i corner_up_left: coordinates of the up-left corner with respect to the (0,0) point of the raw image;
    /// Point2i corner_down_right: coordinates of the down-right corner with respect to the (0,0) point of the raw image;
    /// Point2i calibration_point: x= quote of the vertical black line - 500, y= quote of the horizontal line - 100


    ///LOC HORIZONTAL BLACK LINE
    /// If there exists at least a horizontal line fully contained
    /// by the black horizontal it will be drawn on green on the chessboard;
    /// if that line does not exists a flag will appear on the image;
    ///


    left_chess.crosshair.horizontal=false;
    left_chess.crosshair.vertical=false;
    right_chess.crosshair.horizontal=false;
    right_chess.crosshair.vertical= false;






    extract_relative_dimensions(right_chess, right_src, 0, 1280);

    extract_relative_dimensions(left_chess, left_src, 0, 1280);


    extract_horizontal_black_line(left_chess, left_src);

    extract_horizontal_black_line(right_chess, right_src);


    extract_vertical_black_line(left_chess, left_src);

    extract_vertical_black_line(right_chess, right_src);

    extract_x_correction(right_chess,right_src);

    extract_x_correction(left_chess, left_src);




    for (int i=0; i< right_chess.horizontal_pointlike.size(); i++ )
    {
        circle( right_src.color, Point( (int)right_chess.horizontal_pointlike.at(i).x, (int)right_chess.horizontal_pointlike.at(i).y ) , 1,  CV_RGB(0,255,0), 2, 8, 0 );
    }

    for (int i=0; i< left_chess.horizontal_pointlike.size(); i++ )
    {
        circle( left_src.color, Point( (int)left_chess.horizontal_pointlike.at(i).x, (int)left_chess.horizontal_pointlike.at(i).y ) , 1,  CV_RGB(255,0,0), 2, 8, 0 );
    }

    imshow("RIGHT", right_src.color);
    waitKey(0);

    imshow("LEFT", left_src.color);
    waitKey(0);


    sources left_src_cropped, right_src_cropped;

    adaptive_cut(left_chess, left_src, left_src_cropped, 30);

    adaptive_cut(right_chess, right_src, right_src_cropped, 30);


    extract_chess_limits(left_chess_cropped, left_src_cropped, true, true);

    extract_chess_limits(right_chess_cropped, right_src_cropped, true, true);



    extract_chess_limits(left_chess_cropped, left_src_cropped, true, true);

    extract_chess_limits(right_chess_cropped, right_src_cropped, true, true);


    extract_relative_dimensions (left_chess_cropped, left_src_cropped, 0, left_src_cropped.grayscale.cols );

    extract_relative_dimensions (right_chess_cropped, right_src_cropped, 0, right_src_cropped.grayscale.cols );


    extract_horizontal_black_line(left_chess_cropped, left_src_cropped);

    extract_horizontal_black_line(right_chess_cropped, right_src_cropped);


    extract_vertical_black_line(left_chess_cropped, left_src_cropped);

    extract_vertical_black_line(right_chess_cropped, right_src_cropped);


    extract_x_correction(right_chess_cropped,right_src_cropped);

    extract_x_correction(left_chess_cropped, left_src_cropped);




    ///// IMAGE ENHANCEMENT AND WRITE

    // Set the parameters for the image enhancement

    int kernel_size_lap =7;
    int scale = 1;
    int delta = 0;
    Size ksize= Size(7,7);

    Mat dst, left_refined, right_refined;
    // Image enhancement: Laplacian Sharpening - Gaussian Blur

    double alpha_lap =1.6;
    double beta_lap= -.7;

    double alpha_gauss= 1.6;
    double beta_gauss= -.9;


    Laplacian(left_src_cropped.grayscale, dst, CV_32FC1, kernel_size_lap, scale, delta);
    addWeighted(left_src_cropped.grayscale, alpha_lap, dst, beta_lap, 0.0, left_refined, CV_32FC1);

    GaussianBlur(left_src_cropped.grayscale, dst, ksize, 9.0, BORDER_DEFAULT);
    addWeighted(left_src_cropped.grayscale, alpha_gauss, dst, beta_gauss, 0, left_refined);

    dst.release();

    Laplacian(right_src_cropped.grayscale, dst, CV_32FC1, kernel_size_lap, scale, delta);
    addWeighted(right_src_cropped.grayscale, alpha_lap, dst, beta_lap, 0.0, right_refined, CV_32FC1);

    GaussianBlur(right_src_cropped.grayscale, dst, ksize, 9.0, BORDER_DEFAULT);
    addWeighted(right_src_cropped.grayscale, alpha_gauss, dst, beta_gauss, 0, right_refined);


    destroyAllWindows();


    imwrite("C:/Users/GuidoVitale/Desktop/acquisizioni_v2_0709/tilted_artificiali/synth_cropped/left_tilted_cropped.BMP", left_refined);

    imwrite("C:/Users/GuidoVitale/Desktop/acquisizioni_v2_0709/tilted_artificiali/synth_cropped/right_tilted_cropped.BMP", right_refined);

    return;

}
