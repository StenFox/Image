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

    CImage( int _height, int _width, std::vector<float>& _img );

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

    void setHeight( int _value )
    {
        m_myImage.setColumns( _value );
    }

    int getWidth() const
    {
        return m_myImage.getRows();
    }

    void setWidth( int _value )
    {
        m_myImage.setRows( _value );
    }

    void setPixel( int _columns, int _rows, float _value )
    {
        m_myImage.setItem( _columns, _rows, _value );
    }

    float getPixel( int _columns,int _rows ) const
    {
        return m_myImage.getItem( _columns,_rows );
    }

    void resize( int _columns, int _rows,std::vector<float>& _newImg )
    {
        m_myImage.resize( _columns,_rows,_newImg );
    }

    void oneNormalize()
    {
        m_myImage.oneNormalize();
    }

    void normalizeImage()
    {
        m_myImage.normalize();
    }

private:
    CMatrixV<float> m_myImage;
};

#endif // CIMAGE_H
