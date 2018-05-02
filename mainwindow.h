#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "CImage.h"
#include "CImageHandler.h"
#include "CPyramid.h"

namespace Ui {
class MainWindow;
}

enum TypeChange
{
    rotate,
    shift,
    brightness,
    contrast,
    affin,
    scale,
    noise
};


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_LoadImageButton_clicked();

    void on_GaussBlurButton_clicked();

    void on_SobelButton_clicked();

    void on_pushButton_clicked();

    void on_MoravecButton_clicked();

    void on_HarrisonButton_clicked();

    void on_showOctave_clicked();

    void on_loadImage1_clicked();

    void on_loadImage2_clicked();

    void on_showImages_clicked();

    void on_CompareImageRotate_clicked();

    void on_brightnessChandge_clicked();

    void on_contrastChange_clicked();

    void on_Test1_clicked();

    void on_TestRotate_clicked();

private:
    CImage* myImage;
    CImage* newImage;
    CImage myImage1;
    CImage myImage2;
    CImage temp1;
    CImage temp2;
    CImageHandler myImageHandler;
    CPyramid myPyramidImage;
    Ui::MainWindow *ui;

    float testBrightness( CImage& _myImage );
    float testRotate( CImage& _myImage );
    float testContrast(CImage& _myImage);


    float testImage( float _min, float _max,float _step, CImage& _myImage,TypeChange _type );
    void setInterestPoints(std::vector<QPoint>& _vector,const CImage& _myImage);
};

#endif // MAINWINDOW_H
