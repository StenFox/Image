#ifndef CIMAGE_H
#define CIMAGE_H
#include <vector>
#include <QImage>
#include "QPixmap"
#include "CMatrixV.h"

class CImage
{
public:
    CImage( QString _fileName );
    QImage getImage();
    void setPixel( int _columns,int _rows, int _value );
    float getPixel( int _columns,int _rows )
    {
        return m_myImage.getItem( _columns,_rows );
    }

private:
    CMatrixV<float> m_myImage;
    // Функция перевода изображения в оттенки серого
    void grayScale( QImage _image );
};

#endif // CIMAGE_H
