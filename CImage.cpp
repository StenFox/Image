#include "CImage.h"

using namespace  std;

// Конструктор
CImage::CImage( QString _fileName )
{
    QImage img;
    img.load( _fileName );
    CMatrixV<float> myImage( img.height() ,img.width() );
    m_myImage = myImage;
}

//-----------------------------------------------------------------------------------
CImage::CImage (const CImage & _image)
{
    m_myImage = _image.m_myImage;
}

//-----------------------------------------------------------------------------------
CImage::CImage ( CImage&& _image )
{
   m_myImage = _image.m_myImage;
}

//-----------------------------------------------------------------------------------
CImage& CImage::operator= ( const CImage& _image )
{

}

//-----------------------------------------------------------------------------------
CImage&  CImage::operator= ( CImage&& _image )
{
    if( this != &other )
    {

    }
}

//-----------------------------------------------------------------------------------
CImage::~CImage()
{

}

//-----------------------------------------------------------------------------------
QImage CImage::getImage()
{
    QImage img ( m_myImage.getRows() ,m_myImage.getColumns(), QImage::Format_RGB32 );
    for ( int i = 0; i < m_myImage.getColumns(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( img.scanLine(i) );
        QRgb *end = pixel + m_myImage.getColumns();
        for ( int j =0; pixel != end; pixel++,j++ )
        {
            int gray = m_myImage.getItem(i,j);
            *pixel = QColor( gray, gray, gray ).rgb();
        }
    }
    return img;
}

//-----------------------------------------------------------------------------------
int CImage::getHeight() const
{
    return m_myImage.getColumns();
}

//-----------------------------------------------------------------------------------
int CImage::getWidth() const
{
    return m_myImage.getRows();
}

void CImage::setPixel( int _columns,int _rows, int _value )
{
    m_myImage.setItem( _columns,_rows, _value );
}

float CImage::getPixel( int _columns,int _rows ) const
{
    return m_myImage.getItem(_columns,_rows);
}
