#include <QApplication>
#include <QString>
#include <QFile>
#include <QTime>
#include <QFileDialog>
#include <QStringList>
#include <QList>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <QList>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/opencv.hpp>



#include "main.h"
#include "calib.h"
#include "align.h"
#include <sys/types.h>
#include <sys/stat.h>


using namespace std;


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // Apertura immagine in input e display

    QString dir = QFileDialog::getExistingDirectory(NULL,  QObject::tr("Open Directory where the Images are"),  "/Desktop", QFileDialog::ShowDirsOnly  | QFileDialog::DontResolveSymlinks);
    char* imgs_directory =(char *)malloc(200);
    QByteArray baDir=dir.toLatin1();
    strcpy(imgs_directory,baDir.data());

    QString output_dir = dir+"/Calibration_Folder/SN3/";
    char* output_directory =(char *)malloc(300);
    QByteArray baOutputDir=output_dir.toLatin1();
    strcpy(output_directory,baOutputDir.data());

    if(!QDir(output_dir).exists())// now it works
    {
        QDir().mkpath(output_dir);
    }

    QStringList files =  QFileDialog::getOpenFileNames(NULL, QObject::tr("Select one or more files to open"), dir, QObject::tr("Files (*.bmp)"));


    QString  imageName  = files.QList::value(0); //filen always 1, I have only a chessboard image
    QString str2 ="/";
    int found = imageName.lastIndexOf(str2);
    QString imgs_name = imageName;
    imgs_name.remove(0, found);

    char *imgs_filename=(char *)malloc(50);
    QByteArray ba=imgs_name.toLatin1();
    strcpy(imgs_filename,ba.data());


    Mat inputImage = imread(imageName.toStdString().c_str(), IMREAD_COLOR);
    if (inputImage.empty())
    {
        cout << "ERRORE LETALE: impossibile caricare inputImage." << endl;
        return -1;

    }
    inputImage.release();

    cout<<"Calibration started"<<endl;



    chessboard_properties left_chess, right_chess, left_chess_cropped, right_chess_cropped;




    align (imgs_filename, left_chess, right_chess, left_chess_cropped, right_chess_cropped);

    setup_calibration(imgs_directory, imgs_filename, output_dir, left_chess_cropped, right_chess_cropped );




    ba.clear();
    baDir.clear();
    baOutputDir.clear();

    a.exit();
    return 0;
}
