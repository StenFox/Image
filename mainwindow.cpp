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
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    QGraphicsScene *scene = new QGraphicsScene();
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
    auto pointsImageFirst = myImageHandler.harris( temp1, 397700000, 0.06, true, 100 );
    myImageHandler.descriptorRotation( myImage1, 16, pointsImageFirst );

    temp2 = myImage2;
    myImageHandler.gaussianBlur( 2, temp2, mtBlackEdge );
    auto pointsImageSecond = myImageHandler.harris( temp2, 397700000, 0.06, true, 100 );
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

void MainWindow::on_Test1_clicked()
{
    auto b = testRotate(*myImage);
    int h = 0;
}


float MainWindow::testBrightness( CImage& _myImage )
{
    std::vector<QPoint> pointsImageFirst;
    switch( (mtPointDetector)ui->PointDetector->currentIndex())
    {
        case mtMoravec:
            pointsImageFirst = myImageHandler.moravec( _myImage, 5000, 3, 3, false, 0 );
            break;
        case mtHarrison:
            pointsImageFirst = myImageHandler.harris( _myImage, 397700000, 0.06, true, 100 );
            break;
    }
    float procent = 0;
    for( float curBrightness = ui->MinBrightness->value() ; curBrightness < ui->MaxBrightness->value(); curBrightness += ui->stepBrightnessChange->value()  )
    {
        CImage temp = _myImage;
        myImageHandler.brightnessChange( temp, curBrightness );
        temp.normalizeImage();
        std::vector<QPoint> pointsImageSecond;
        switch( (mtPointDetector)ui->PointDetector->currentIndex())
        {
            case mtMoravec:
                pointsImageSecond = myImageHandler.moravec( temp, 5000, 3, 3, false, 0 );
                break;
            case mtHarrison:
                pointsImageSecond = myImageHandler.harris( temp, 397700000, 0.06, true, 100 );
                break;
        }
        int col = 0;
        for( int i = 0; i < pointsImageFirst.size(); i++ )
        {
            auto p1 = pointsImageFirst[i];
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
        }
        if( col != 0 )
            procent += ( (float)col / pointsImageFirst.size() ) * 100;
        else
            procent /= 2;
    }
    return procent;
}


float MainWindow::testContrast(CImage& _myImage)
{
    std::vector<QPoint> pointsImageFirst;
    switch( (mtPointDetector)ui->PointDetector->currentIndex())
    {
        case mtMoravec:
            pointsImageFirst = myImageHandler.moravec( _myImage, 5000, 3, 3, false, 0 );
            break;
        case mtHarrison:
            pointsImageFirst = myImageHandler.harris( _myImage, 397700000, 0.06, true, 100 );
            break;
    }
    float procent = 0;
    for( float curContrast = ui->MinContrast->value() ; curContrast < ui->MaxContrast->value(); curContrast += ui->stepContrastChange->value()  )
    {
        CImage temp = _myImage;
        myImageHandler.contrastChange( temp, curContrast );
        temp.normalizeImage();
        std::vector<QPoint> pointsImageSecond;
        switch( (mtPointDetector)ui->PointDetector->currentIndex())
        {
            case mtMoravec:
                pointsImageSecond = myImageHandler.moravec( temp, 5000, 3, 3, false, 0 );
                break;
            case mtHarrison:
                pointsImageSecond = myImageHandler.harris( temp, 397700000, 0.06, true, 100 );
                break;
        }
        int col = 0;
        for( int i = 0; i < pointsImageFirst.size(); i++ )
        {
            auto p1 = pointsImageFirst[i];
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
        }
        if( col != 0 )
            procent += ( (float)col / pointsImageFirst.size() ) * 100;
        else
            procent /= 2;
    }
    return procent;
}

float MainWindow::testRotate( CImage& _myImage )
{
    std::vector<QPoint> pointsImageFirst;
    switch( (mtPointDetector)ui->PointDetector->currentIndex())
    {
        case mtMoravec:
            pointsImageFirst = myImageHandler.moravec( _myImage, 5000, 3, 3, false, 0 );
            break;
        case mtHarrison:
            pointsImageFirst = myImageHandler.harris( _myImage, 397700000, 0.06, false, 100 );
            break;
    }
    float procent = 0;
    int count = 0;
    for( float curAngle = ui->MinAgleRotate->value() ; curAngle < ui->MaxAgleRotate->value(); curAngle += ui->stepAgleRotateChange->value()  )
    {
        QImage qtemp = _myImage.getImage();
        QTransform transform;
        transform.rotate( curAngle );
        qtemp = qtemp.transformed(transform);
        transform = QImage::trueMatrix( transform, _myImage.getWidth(), _myImage.getHeight() );
        CImage temp( qtemp.height(), qtemp.width() );
        myImageHandler.grayScale( qtemp, temp );
        std::vector<QPoint> pointsImageSecond;
        switch( (mtPointDetector)ui->PointDetector->currentIndex())
        {
            case mtMoravec:
                pointsImageSecond = myImageHandler.moravec( temp, 5000, 3, 3, false, 0 );
                break;
            case mtHarrison:
                pointsImageSecond = myImageHandler.harris( temp, 397700000, 0.06, false, 100 );
                break;
        }
        int col = 0;
        for( int i = 0; i < pointsImageFirst.size(); i++ )
        {
            auto p1 = pointsImageFirst[i];
            p1 = transform.map( p1 );
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
        }
        procent += ( (float)col / pointsImageFirst.size() ) * 100;
        count++;
    }
    return procent / count;
}

float MainWindow::testImage( float _min, float _max,float _step, CImage& _myImage,TypeChange _type )
{
    std::vector<QPoint> pointsImageFirst;
    switch( (mtPointDetector)ui->PointDetector->currentIndex())
    {
        case mtMoravec:
            pointsImageFirst = myImageHandler.moravec( _myImage, 5000, 3, 3, false, 0 );
            break;
        case mtHarrison:
            pointsImageFirst = myImageHandler.harris( _myImage, 397700000, 0.06, false, 100 );
            break;
    }
    float procent = 0;
    int count = 0;
    for( float value = _min ; value < _max; value += _step  )
    {
        QImage qtemp = _myImage.getImage();
        QTransform transform;
        CImage temp;
        switch (_type)
        {
        case rotate:
        {
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
            myImageHandler.addNoise( temp );
            temp.normalizeImage();
            break;
        }

        case scale:
        {
            transform.scale(1,1);
            qtemp = qtemp.transformed(transform);
            transform = QImage::trueMatrix( transform, _myImage.getWidth(), _myImage.getHeight() );
            temp.setSize( qtemp.height(), qtemp.width() );
            myImageHandler.grayScale( qtemp, temp );
        }

        }
        std::vector<QPoint> pointsImageSecond;
        setInterestPoints(pointsImageSecond,temp);
        switch( (mtPointDetector)ui->PointDetector->currentIndex())
        {
            case mtMoravec:
                pointsImageSecond = myImageHandler.moravec( temp, 5000, 3, 3, false, 0 );
                break;
            case mtHarrison:
                pointsImageSecond = myImageHandler.harris( temp, 397700000, 0.06, false, 100 );
                break;
        }
        int col = 0;
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
                p1.setX(p1.x() + 1);
                p1.setY(p1.y() + 1);
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
        }
        procent += ( (float)col / pointsImageFirst.size() ) * 100;
        count++;
    }
    return procent / count;
}

void MainWindow::setInterestPoints(std::vector<QPoint>& _vector,const CImage& _myImage)
{
    switch( (mtPointDetector)ui->PointDetector->currentIndex())
    {
        case mtMoravec:
            _vector = myImageHandler.moravec( _myImage, 5000, 3, 3, false, 0 );
            break;
        case mtHarrison:
            _vector = myImageHandler.harris( _myImage, 397700000, 0.06, false, 100 );
            break;
    }
}

void MainWindow::on_TestRotate_clicked()
{
    float result = testRotate(*myImage);
    QMessageBox::information(this,"Результат","Устойчив на " + QString::number( result ) + " процентов к поворотам изображения",QMessageBox::StandardButton::Ok);
}
