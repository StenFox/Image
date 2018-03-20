#include "CImage.h"
#include "CImageKernels.h"

using namespace  std;

//-----------------------------------------------------------------------------------
CImage::CImage( QString _fileName )
{
    QImage img;
    img.load( _fileName );
    m_myImage( img.height(),img.width() )
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
