#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "CImage.h"

namespace Ui {
class MainWindow;
}

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

private:
    CImage *myImage;
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
