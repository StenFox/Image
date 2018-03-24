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
    CImage( int _height, int _width );

    // Контсрутор копирования
    CImage (const CImage & _image);

    // Контсрутор перемещения
    CImage ( CImage&& _image );

    // Оператор присваивания
    CImage& operator= ( const CImage& _image );

    // Оператор перемещения
    CImage& operator= ( CImage&& _image );

    // Деструктор
    ~CImage();

    QImage getImage();

    int getHeight() const
    {
        return m_myImage.getColumns();
    }

    int getWidth() const
    {
        return m_myImage.getRows();
    }

    void setPixel( int _columns,int _rows, int _value )
    {
        m_myImage.setItem( _columns,_rows, _value );
    }

    float getPixel( int _columns,int _rows ) const
    {
        return m_myImage.getItem( _columns,_rows );
    }
private:
    CMatrixV<float> m_myImage;
};

#endif // CIMAGE_H
