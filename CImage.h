#ifndef CIMAGE_H
#define CIMAGE_H
#include <vector>
#include <QImage>
#include "QPixmap"
#include "CMatrixV.h"

class CImage
{
public:
    // Конструктор
    CImage( QString _fileName );

    // Контсрутор копирования
    CImage (const CImage & _image);

    // Контсрутор перемещения
    CImage ( CImage&& _image );

    // Оператор присваивания
    CImage& operator= ( const CImage _image );

    // Оператор перемещения
    CImage& operator= ( CImage&& _image );

    // Деструктор
    ~CImage();

    QImage getImage();

    int getHeight() const;

    int getWidth() const;

    void setPixel( int _columns,int _rows, int _value );

    float getPixel( int _columns,int _rows ) const;

private:
    CMatrixV<float> m_myImage;
};

#endif // CIMAGE_H
