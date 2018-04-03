#include "CImage.h"

using namespace  std;

// Конструктор
CImage::CImage( int _height, int _width )
{
    CMatrixV<float> myImage( _height, _width );
    m_myImage = std::move( myImage );
}

CImage::CImage( int _height, int _width ,vector<float>& _img )
{
    CMatrixV<float> myImage( _height, _width, _img );
    m_myImage = std::move( myImage );
}

//-----------------------------------------------------------------------------------
CImage::CImage (const CImage & _image)
{
    m_myImage = _image.m_myImage;
}

//-----------------------------------------------------------------------------------
CImage::CImage ( CImage&& _image )
{
   m_myImage = std::move( _image.m_myImage );
}

//-----------------------------------------------------------------------------------
CImage& CImage::operator= ( const CImage& _image )
{
    m_myImage = _image.m_myImage;
    return *this;
}

//-----------------------------------------------------------------------------------
CImage&  CImage::operator= ( CImage&& _image )
{
    if (this != &_image)
    {
        m_myImage.getMatrix().clear();
        m_myImage = std::move( _image.m_myImage );
    }
    return *this;
}

//-----------------------------------------------------------------------------------
CImage::~CImage()
{

}

//-----------------------------------------------------------------------------------
QImage CImage::getImage()
{
    QImage img ( m_myImage.getRows() ,m_myImage.getColumns(), QImage::Format_RGB32 );
    m_myImage.normalize();
    for ( int i = 0; i < m_myImage.getColumns(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( img.scanLine(i) );
        QRgb *end = pixel + m_myImage.getColumns();
        for ( int j =0; pixel != end; pixel++,j++ )
        {
            int item = m_myImage.getItem(i,j);
            *pixel = QColor( item, item, item ).rgb();
        }
    }
    return img;
}
