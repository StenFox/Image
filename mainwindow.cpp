#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QFileDialog"
#include "QGraphicsScene"
#include "QGraphicsPixmapItem"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->M_EdgeComboBox->insertItem( 0, "'Чёрные' границы" );
    ui->M_EdgeComboBox->insertItem( 1, "Копирование границ" );
    ui->M_EdgeComboBox->insertItem( 2, "Тор" );
    myImageHandler = new CImageHandler();
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
    myImageHandler->grayScale( img ,*myImage );
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

void MainWindow::on_GaussBlurButton_clicked()
{
    myImageHandler->gaussianBlur( ui->doubleSpinBox->value(),  *myImage, ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex() );
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

void MainWindow::on_SobelButton_clicked()
{
    myImageHandler->sobel( ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex(), *myImage );
    ////ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(myImage->getImage()));
    scene->addItem(item);
    ui->graphicsView->setScene(scene);
}

void MainWindow::on_ReloadImageButton_clicked()
{

}

void MainWindow::on_pushButton_clicked()
{
    myPyramidImage = myImageHandler->gaussPyramid( *myImage,ui->colOctave->value(),ui->colScales->value(),ui->doubleSpinBox->value() );
}

void MainWindow::on_MoravecButton_clicked()
{
    ui->graphicsView->scene()->clear();
    QGraphicsScene *scene = new QGraphicsScene();
    QGraphicsPixmapItem* item = new QGraphicsPixmapItem( QPixmap::fromImage( myImage->getImage() ));
    scene->addItem( item );
    bool useNonMax = ui->useNonMaximum->checkState() == Qt::Checked;
    auto points = myImageHandler->moravec( *myImage,1800, 3, 3, useNonMax, 500 );

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
            myImageHandler->gaussianBlur(ui->sigmaForHarrison->value(), *myImage ,( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex());
    bool useNonMax = ui->useNonMaximum->checkState() == Qt::Checked;
    auto points = myImageHandler->harris( *myImage, ui->Tvalue->value(), 0.06,useNonMax, 500 );
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
    QImage img;
    img.load( fileName );
    myImage1 = new CImage( img.height(), img.width() );
    myImageHandler->grayScale( img ,*myImage1 );
}

void MainWindow::on_loadImage2_clicked()
{
    QString fileName = QFileDialog::getOpenFileName( this,"Open Image",nullptr,"Image files (*.png *.jpg *.bmp)" );
    QImage img;
    img.load( fileName );
    myImage2 = new CImage( img.height(), img.width() );
    myImageHandler->grayScale( img ,*myImage2 );
}

void MainWindow::on_showImages_clicked()
{
    QGraphicsScene *scene = new QGraphicsScene( 0, 0, myImage1->getWidth() + myImage2->getWidth(), ui->graphicsView->height() );
    QGraphicsPixmapItem* item1 = new QGraphicsPixmapItem( QPixmap::fromImage( myImage1->getImage() ) );
    QGraphicsPixmapItem* item2 = new QGraphicsPixmapItem( QPixmap::fromImage( myImage2->getImage() ) );
    item2->setOffset( myImage1->getWidth(),0 );
    scene->addItem( item1 );
    scene->addItem( item2 );

    auto des = myImageHandler->imageComparison( *myImage1, *myImage2 );

    QPen pen;
    pen.setBrush( QBrush() );
    pen.setColor( QColor(255,0,0));
    for( size_t i = 0; i< des.size(); i++)
    {
        scene->addLine(des[i].first.getInterestPoint().x(),des[i].first.getInterestPoint().y(),des[i].second.getInterestPoint().x() + myImage1->getWidth() ,des[i].second.getInterestPoint().y(),pen );
    }

    ui->graphicsView->setScene( scene );
}
