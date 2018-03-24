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
    ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
    ui->label_2->setPixmap( QPixmap::fromImage( img ) );
}

void MainWindow::on_GaussBlurButton_clicked()
{
    myImageHandler->gaussianBlur( ui->doubleSpinBox->value(),  *myImage, ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex() );
    ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
}

void MainWindow::on_SobelButton_clicked()
{
    myImageHandler->sobel( ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex(), *myImage );
    ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
}

void MainWindow::on_ReloadImageButton_clicked()
{

}
