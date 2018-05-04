#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QFileDialog"
#include "QGraphicsScene"
#include "QGraphicsPixmapItem"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->M_EdgeComboBox->insertItem( 0, "'Чёрные' границы" );
    ui->M_EdgeComboBox->insertItem( 1, "Копирование границ" );
    ui->M_EdgeComboBox->insertItem( 2, "Тор" );

    ui->PointDetector->insertItem( mtMoravec, "Моравик" );
    ui->PointDetector->insertItem( mtHarrison, "Харисон" );

    ui->DescriptorsList->insertItem( 0, "Самый простой дескриптор" );
    ui->DescriptorsList->insertItem( 1, "Устойчивый к вращению дескритор" );
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_LoadImageButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName( this,"Open Image",nullptr,"Image files (*.png *.jpg *.bmp)" );
    QImage img;
    img.load(fileName);

    myImage = new CImage( img.height(), img.width() );
    myImageHandler.grayScale( img ,*myImage );

    //myImageHandler.addNoise(*myImage,100);

    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

void MainWindow::on_GaussBlurButton_clicked()
{
    myImageHandler.gaussianBlur( ui->doubleSpinBox->value(),  *myImage, ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex() );
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

void MainWindow::on_SobelButton_clicked()
{
    myImageHandler.sobel( ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex(), *myImage );
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

void MainWindow::on_pushButton_clicked()
{
    myPyramidImage = myImageHandler.gaussPyramid( *myImage,ui->colOctave->value(),ui->colScales->value(),ui->doubleSpinBox->value() );
}

void MainWindow::on_MoravecButton_clicked()
{
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem( QPixmap::fromImage( myImage->getImage() ));
    scene->addItem( item );
    bool useNonMax = ui->useNonMaximum->checkState() == Qt::Checked;
    auto points = myImageHandler.moravec( *myImage,1800, 3, 3, useNonMax, 500 );

    QPen pen;
    pen.setColor(Qt::GlobalColor::red);
    for( size_t i = 0; i < points.size(); i++ )
    {
        scene->addEllipse( points[i].x(), points[i].y(), 1, 1, pen );
    }

    ui->graphicsView->setScene( scene );
}

void MainWindow::on_HarrisonButton_clicked()
{
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    if( ui->useGauss->checkState() == Qt::Checked )
            myImageHandler.gaussianBlur(ui->sigmaForHarrison->value(), *myImage ,( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex());
    bool useNonMax = ui->useNonMaximum->checkState() == Qt::Checked;
    auto points = myImageHandler.harris( *myImage, ui->Tvalue->value(), 0.06,useNonMax, 100 );
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage( myImage->getImage() ) );
    scene->addItem( item );

    QPen pen;
    pen.setColor(Qt::GlobalColor::red);
    for( size_t i = 0; i < points.size(); i++ )
    {
        scene->addEllipse( points[i].x(), points[i].y(), 1, 1, pen );
    }

    ui->graphicsView->setScene( scene );
}

void MainWindow::on_showOctave_clicked()
{
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage( myPyramidImage.getImageInOctaves( ui->octaves->value() ).getImage() ) );
    scene->addItem( item );
    ui->graphicsView->setScene( scene );
}

void MainWindow::on_loadImage1_clicked()
{
    QString fileName = QFileDialog::getOpenFileName( this,"Open Image",nullptr,"Image files (*.png *.jpg *.bmp)" );
    if(!fileName.isEmpty())
    {
        QImage img;
        img.load( fileName );
        CImage myimg( img.height(), img.width() );
        myImageHandler.grayScale( img ,myimg );
        myImage1 = std::move( myimg );
        QTransform t;
        t.rotate(60);
        img = img.transformed(t);
        CImage myimg1( img.height(), img.width() );
        myImageHandler.grayScale( img ,myimg1 );
        myImage2 = std::move( myimg1 );
    }
}

void MainWindow::on_loadImage2_clicked()
{
    QString fileName = QFileDialog::getOpenFileName( this,"Open Image",nullptr,"Image files (*.png *.jpg *.bmp)" );
    if(!fileName.isEmpty())
    {
        QImage img;
        img.load( fileName );
        CImage myimg( img.height(), img.width() );
        myImageHandler.grayScale( img ,myimg );
        myImage2 = std::move( myimg );
    }
}

void MainWindow::on_showImages_clicked()
{
    QGraphicsScene *scene = new QGraphicsScene( 0, 0, myImage1.getWidth() + myImage2.getWidth(), ui->graphicsView->height() );
    QGraphicsPixmapItem* item1 = new QGraphicsPixmapItem( QPixmap::fromImage( myImage1.getImage() ) );
    QGraphicsPixmapItem* item2 = new QGraphicsPixmapItem( QPixmap::fromImage( myImage2.getImage() ) );
    item2->setOffset( myImage1.getWidth(),0 );
    scene->addItem( item1 );
    scene->addItem( item2 );

    auto pointsImageFirst = myImageHandler.moravec( myImage1, 5000, 3, 3, false, 0 );
    myImageHandler.descriptor( myImage1, 16, 8, 16, pointsImageFirst );

    auto pointsImageSecond = myImageHandler.moravec( myImage2, 5000, 3, 3, false, 0 );
    myImageHandler.descriptor( myImage2, 16, 8, 16, pointsImageSecond );

    auto des = myImageHandler.imageComparison( myImage1, myImage2, true, 0.9 );

    QPen pen;
    pen.setBrush( QBrush() );

    pen.setColor( QColor(255,0,0));
    for( size_t i = 0; i< des.size(); i++)
    {
        auto r = rand() % 255 + 1;
        auto g = rand() % 255 + 1;
        auto b = rand() % 255 + 1;
        pen.setColor( QColor(r,g,b));
        scene->addLine(des[i].first.getInterestPoint().x(),des[i].first.getInterestPoint().y(),des[i].second.getInterestPoint().x() + myImage1.getWidth() ,des[i].second.getInterestPoint().y(),pen );
    }

    ui->graphicsView->setScene( scene );
}

void MainWindow::on_CompareImageRotate_clicked()
{
    QGraphicsScene *scene = new QGraphicsScene( 0, 0, myImage1.getWidth() + myImage2.getWidth(), ui->graphicsView->height() );
    QGraphicsPixmapItem* item1 = new QGraphicsPixmapItem( QPixmap::fromImage( myImage1.getImage() ) );
    QGraphicsPixmapItem* item2 = new QGraphicsPixmapItem( QPixmap::fromImage( myImage2.getImage() ) );
    item2->setOffset( myImage1.getWidth(),0 );
    scene->addItem( item1 );
    scene->addItem( item2 );

    temp1 = myImage1;
    myImageHandler.gaussianBlur( 2, temp1, mtBlackEdge );
    auto pointsImageFirst = myImageHandler.harris( temp1, 397700000, 0.06, true, 400 );
    myImageHandler.descriptorRotation( myImage1, 16, pointsImageFirst );

    temp2 = myImage2;
    myImageHandler.gaussianBlur( 2, temp2, mtBlackEdge );
    auto pointsImageSecond = myImageHandler.harris( temp2, 397700000, 0.06, true, 400 );
    myImageHandler.descriptorRotation( myImage2, 16, pointsImageSecond );

    auto des = myImageHandler.imageComparison( myImage1, myImage2, false, 0.8 );

    QPen pen;
    pen.setBrush( QBrush() );

    pen.setColor( QColor(255,0,0) );
    for( size_t i = 0; i < des.size(); i++)
    {
        auto r = rand() % 255 + 1;
        auto g = rand() % 255 + 1;
        auto b = rand() % 255 + 1;
        pen.setColor( QColor(r,g,b));
        scene->addLine(des[i].first.getInterestPoint().x(),des[i].first.getInterestPoint().y(),des[i].second.getInterestPoint().x() + myImage1.getWidth() ,des[i].second.getInterestPoint().y(),pen );
    }

    ui->graphicsView->setScene( scene );
}

void MainWindow::on_brightnessChandge_clicked()
{
    myImageHandler.brightnessChange(*myImage,250);
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}



void MainWindow::on_contrastChange_clicked()
{
    myImageHandler.contrastChange(*myImage,250);
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

float MainWindow::testImage( float _min, float _max,float _step, CImage& _myImage,TypeChange _type, bool _testDes )
{
    std::vector<QPoint> pointsImageFirst;
    setInterestPoints( pointsImageFirst, _myImage );
    std::vector<QPoint> points;
    for( int y = 0; y < _myImage.getHeight(); y++ )
    {
        for( int x = 0; x < _myImage.getWidth(); x++ )
        {
            if( x==0 )
                points.push_back( QPoint(x,y) );
            else if( y == 0)
                points.push_back( QPoint(x,y) );
            else if(x == _myImage.getWidth() - 1 )
                points.push_back( QPoint(x,y) );
            else if(y == _myImage.getHeight() - 1 )
                points.push_back( QPoint(x,y) );
        }
    }

    if(_testDes)
    {
        if(ui->DescriptorsList->currentIndex() == 0)
            myImageHandler.descriptor( _myImage, 16, 8, 16, pointsImageFirst );
        else
            myImageHandler.descriptorRotation( _myImage, 16, pointsImageFirst );
    }
    float procent = 0;
    int count = 0;
    for( float value = _min ; value < _max; value += _step  )
    {
        QTransform transform;
        CImage temp = transformImage(_myImage, _type, value, transform );
        std::vector<QPoint> pointsImageSecond;
        setInterestPoints( pointsImageSecond,temp );
        int col = 0;
        if( _testDes )
        {
            if(ui->DescriptorsList->currentIndex() == 0)
                myImageHandler.descriptor( temp, 16, 8, 16, pointsImageSecond );
            else
                myImageHandler.descriptorRotation( temp, 16, pointsImageSecond );
            auto des = myImageHandler.imageComparison( _myImage, temp, true, 0.8 );
            if( des.size() != 0 )
            {
                col = compareDes(des,_type,transform,value);
                procent += ( (float)col / des.size() ) * 100;
            }
        }
        else
        {
            for( int i = 0; i < pointsImageFirst.size(); i++ )
            {
                auto p1 = pointsImageFirst[i];
                switch (_type)
                {
                case scale:
                case affin:
                case rotate:
                {
                    p1 = transform.map( p1 );
                    break;
                }
                case shift:
                {
                    p1.setX( p1.x() + value );
                    p1.setY( p1.y() + value );
                    break;
                }
                }
                for( int j = 0; j < pointsImageSecond.size(); j++ )
                {
                    auto p2 = pointsImageSecond[j];
                    if( abs( p1.x() - p2.x() ) <= 2 && abs( p1.y() - p2.y() ) <= 2 )
                    {
                        col++;
                        pointsImageSecond.erase( pointsImageSecond.begin() + j );
                        break;
                    }
                }
                procent += ( (float)col / pointsImageFirst.size() ) * 100;
                col = 0;
            }
        }
        count++;
    }
    return procent / count;
}

void MainWindow::filtrate(std::vector<QPoint>& _filtr,std::vector<QPoint>& _points, QTransform& transform)
{
    for( int y = 0; y < _filtr.size(); y++ )
    {
        auto p1 = _filtr[y];
        p1 = transform.map(p1);
        for( int x = 0; x < _points.size(); x++ )
        {
            auto p2 = _points[x];
            if( abs( p1.x() - p2.x() ) <= 2 && abs( p1.y() - p2.y() ) <= 2 )
            {
                _points.erase(_points.begin() + x);
                break;
            }

        }
    }
}

void MainWindow::setInterestPoints(std::vector<QPoint>& _vector,CImage& _myImage)
{
    switch( (mtPointDetector)ui->PointDetector->currentIndex())
    {
        case mtMoravec:
            _vector = myImageHandler.moravec( _myImage, 5000, 3, 3, false, 0 );
            break;
        case mtHarrison:
            CImage temp = _myImage;
            myImageHandler.gaussianBlur( 2, temp, mtBlackEdge );
            _vector = myImageHandler.harris( temp, 497700000, 0.06, true, 500 );
            break;
    }
}

//-----------------------------------------------------------------------------------
int MainWindow::compareDes( const std::vector<std::pair<CDescriptor,CDescriptor>>& des,TypeChange _type, QTransform& _trasform, float value )
{
    int col = 0;
    for( int i = 0; i < des.size(); i++ )
    {
        QPoint p1 = des[i].first.getInterestPoint();
        QPoint p2 = des[i].second.getInterestPoint();
        switch (_type)
        {
        case scale:
        case affin:
        case rotate:
        {
            p1 = _trasform.map( p1 );
            break;
        }
        case shift:
        {
            p1.setX(p1.x() + value );
            p1.setY(p1.y() + value );
            break;
        }
        }
        if( abs( p1.x() - p2.x() ) <= 16 && abs( p1.y() - p2.y() ) <= 16 )
        {
            col++;
        }
    }
    return col;
}

CImage MainWindow::transformImage(CImage& _myImage,TypeChange _type, float value, QTransform& transform)
{
    CImage temp;
    switch (_type)
    {
        case rotate:
        {
            QImage qtemp = _myImage.getImage();
            transform.rotate( value );
            qtemp = qtemp.transformed(transform);
            transform = QImage::trueMatrix( transform, _myImage.getWidth(), _myImage.getHeight() );
            temp.setSize( qtemp.height(), qtemp.width() );
            myImageHandler.grayScale( qtemp, temp );
            break;
        }
        case brightness:
        {
            temp = _myImage;
            myImageHandler.brightnessChange( temp, value );
            temp.normalizeImage();
            break;
        }
        case contrast:
        {
            temp = _myImage;
            myImageHandler.contrastChange( temp, value );
            temp.normalizeImage();
            break;
        }
        case shift:
        {
            temp = _myImage;
            myImageHandler.shiftImage( temp, value, value );
            temp.normalizeImage();
            break;
        }
        case affin:
        {
            QImage qtemp = _myImage.getImage();
            transform.setMatrix(ui->m11->value(),ui->m12->value(),ui->m13->value()
                                ,ui->m21->value(),ui->m22->value(),ui->m23->value()
                                ,ui->m31->value(),ui->m32->value(),ui->m33->value());
            qtemp = qtemp.transformed(transform);
            transform = QImage::trueMatrix( transform, _myImage.getWidth(), _myImage.getHeight() );
            temp.setSize( qtemp.height(), qtemp.width() );
            myImageHandler.grayScale( qtemp, temp );
            break;
        }
        case noise:
        {
            temp = _myImage;
            myImageHandler.addNoise( temp, value );
            temp.normalizeImage();
            break;
        }

        case scale:
        {
            QImage qtemp = _myImage.getImage();
            transform.scale(value,value);
            qtemp = qtemp.transformed(transform);
            transform = QImage::trueMatrix( transform, _myImage.getWidth(), _myImage.getHeight() );
            temp.setSize( qtemp.height(), qtemp.width() );
            myImageHandler.grayScale( qtemp, temp );
            break;
        }
    }
    return temp;
}

void MainWindow::on_TestRotate_clicked()
{
    float result = testImage(ui->MinAgleRotate->value(),ui->MaxAgleRotate->value(),ui->stepAgleRotateChange->value(),*myImage,rotate, ui->TestDescriptors->isChecked() );
    QMessageBox::information(this,"Результат","Устойчив на " + QString::number( result ) + " процентов к поворотам изображения",QMessageBox::StandardButton::Ok);
}

void MainWindow::on_TestNoise_clicked()
{
    float result = testImage(ui->MinNoise->value(),ui->MaxNoise->value(),ui->stepNoiseChange->value(),*myImage, noise, ui->TestDescriptors->isChecked() );
    QMessageBox::information(this,"Результат","Устойчив на " + QString::number( result ) + " процентов к шуму",QMessageBox::StandardButton::Ok);
}

void MainWindow::on_TestScale_clicked()
{
    float result = testImage( ui->MinScale->value(), ui->MaxScale->value(), ui->stepScaleChange->value(), *myImage, scale, ui->TestDescriptors->isChecked() );
    QMessageBox::information( this,"Результат","Устойчив на " + QString::number( result ) + " процентов к изменению масштаба",QMessageBox::StandardButton::Ok );
}

void MainWindow::on_TestBrightness_clicked()
{
    float result = testImage( ui->MinBrightness->value(), ui->MaxBrightness->value(), ui->stepBrightnessChange->value(), *myImage, brightness, ui->TestDescriptors->isChecked() );
    QMessageBox::information( this,"Результат","Устойчив на " + QString::number( result ) + " процентов к изменению яркости", QMessageBox::StandardButton::Ok );
}

void MainWindow::on_TestContrast_clicked()
{
    float result = testImage( ui->MinContrast->value(), ui->MaxContrast->value(), ui->stepContrastChange->value(), *myImage, contrast, ui->TestDescriptors->isChecked() );
    QMessageBox::information( this,"Результат","Устойчив на " + QString::number( result ) + " процентов к изменению контраста", QMessageBox::StandardButton::Ok );
}

void MainWindow::on_TestShift_clicked()
{
    float result = testImage( ui->MinShiftX->value(), ui->MaxShiftX->value(), ui->stepShiftChangeX->value(), *myImage, shift, ui->TestDescriptors->isChecked() );
    QMessageBox::information( this,"Результат","Устойчив на " + QString::number( result ) + " процентов к сдвигам", QMessageBox::StandardButton::Ok );
}

void MainWindow::on_TestContrast_3_clicked()
{
    float result = testImage( 0, 1, 1, *myImage, affin, ui->TestDescriptors->isChecked() );
    QMessageBox::information( this,"Результат","Устойчив на " + QString::number( result ) + " процентов к аффинным преобразованиям", QMessageBox::StandardButton::Ok );
}
