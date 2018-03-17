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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_LoadImageButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,"Open Image",nullptr,"Image files (*.png *.jpg *.bmp)");
    myImage = new CImage(fileName);
    ui->label->setPixmap( QPixmap::fromImage(myImage->getImage()));
    ui->label_2->setPixmap( QPixmap::fromImage(myImage->getOriginalImage()));
}

void MainWindow::on_GaussBlurButton_clicked()
{
    myImage->GaussianBlur(ui->doubleSpinBox->value());
    ui->label->setPixmap( QPixmap::fromImage(myImage->getImage()));
}

void MainWindow::on_SobelButton_clicked()
{
    myImage->Sobel();
    ui->label->setPixmap( QPixmap::fromImage(myImage->getImage()));
}
