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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_LoadImageButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName( this,"Open Image",nullptr,"Image files (*.png *.jpg *.bmp)" );
    myImage = new CImage( fileName );
    //ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
    //ui->label_2->setPixmap( QPixmap::fromImage( myImage->getOriginalImage() ) );
}

void MainWindow::on_GaussBlurButton_clicked()
{
    //myImage->gaussianBlur( ui->doubleSpinBox->value(), ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex() );
    //ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
}

void MainWindow::on_SobelButton_clicked()
{
    //myImage->sobel( ( mtProcessingEdgeEffects )ui->M_EdgeComboBox->currentIndex() );
    //ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
}

void MainWindow::on_ReloadImageButton_clicked()
{
    //myImage->reloadImage();
   // ui->label->setPixmap( QPixmap::fromImage( myImage->getImage() ) );
}
