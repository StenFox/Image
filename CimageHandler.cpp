#include "CImageHandler.h"

using namespace  std;

CMatrixV<int> CImage::g_sobelX( 3, 3, VsobelX );
CMatrixV<int> CImage::g_sobelY( 3, 3, VsobelY );

CMatrixV<int> CImage::g_prewittX( 3, 3, VprewittX );
CMatrixV<int> CImage::g_prewittY( 3, 3, VprewittY );

CMatrixV<int> CImage::g_robertX( 3, 3, VrobertX );
CMatrixV<int> CImage::g_robertY( 3, 3, VrobertY );

//-----------------------------------------------------------------------------------
CImageHandler::CImageHandler()
{

}

//-----------------------------------------------------------------------------------
void CImageHandler::grayScale(QImage _image)
{
    Q_ASSERT( _image.format() == QImage::Format_RGB32 );
    for( int i = 0; i < _image.height(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( _image.scanLine( i ) );
        QRgb *end = pixel + _image.width();
        for(auto j = 0 ; pixel != end; pixel++,j++ )
        {
            int gray = qGray( *pixel );
            m_myImage->setPixel( i,j,gray );
        }
    }
}

//-----------------------------------------------------------------------------------
void CImageHandler::sobel( mtProcessingEdgeEffects _method )
{
    magnitude( m_myImage, convolution( g_sobelX, _method ), convolution( g_sobelY, _method ) );
}

//-----------------------------------------------------------------------------------
void CImageHandler::priwitt( mtProcessingEdgeEffects _method )
{
    magnitude( m_myImage, convolution( g_prewittX, _method ), convolution( g_prewittY, _method ) );
}

//-----------------------------------------------------------------------------------
void CImageHandler::robert( mtProcessingEdgeEffects _method )
{
    magnitude( m_myImage, convolution( g_robertX, _method ), convolution( g_robertY, _method ) );
}

//-----------------------------------------------------------------------------------
void CImageHandler::resizeTwo()
{
    m_myImage = resizeBilinear( m_myImage, m_width, m_height, m_width/2, m_height/2 );
    m_height /= 2;
    m_width /= 2;
}

//-----------------------------------------------------------------------------------
void CImageHandler::magnitude(vector<int>& _input, const vector<int>& _gx, const vector<int>& _gy)
{
    for ( auto y = 0; y < m_height; y++ )
    {
        for ( auto x = 0; x < m_width; x++ )
        {
            _input[ y * m_width + x ] = qBound( 0x00, static_cast<int>( hypot ( _gx[ y * m_width + x ], _gy[ y * m_width + x ] ) ), 0xFF );
        }
    }
}

//-----------------------------------------------------------------------------------
void CImageHandler::gaussianBlur( float _sigma, mtProcessingEdgeEffects _method )
{
    convolutionForGauss( _sigma, _method );
}

//-----------------------------------------------------------------------------------
float CImageHandler::gaussian( int _x,float _s )
{
    return  exp( -( _x * _x ) / _s ) / _s / M_PI;
}

//-----------------------------------------------------------------------------------
vector<float> CImageHandler::gaussianKernel( float _sigma )
{
    unsigned sizeKernel = 3 * _sigma;

    // ядро меньше 2 не имеет смысла
    if( sizeKernel < 1 )
        sizeKernel = 3;

    // обеспечиваем нечётность размерности ядра
    if( sizeKernel % 2 == 0 )
        sizeKernel++;

    // Резервируем память
    vector<float> gaussKernel1D;

    gaussKernel1D.reserve( sizeKernel );
    int edgeKernel = sizeKernel/2;

    float sum = 0;
    float s = 2 * _sigma * _sigma;

    for ( int x = -edgeKernel ; x <= edgeKernel; x++ )
    {
        auto temp = gaussian( x,s );
        gaussKernel1D.push_back( temp );
        sum += temp;
    }

    // Нормализуем
    for ( size_t i = 0; i < gaussKernel1D.size(); i++ )
    {
        gaussKernel1D[i] /= sum;
    }

    return gaussKernel1D;
}

//-----------------------------------------------------------------------------------
void CImageHandler::convolutionForGauss( float _sigma, mtProcessingEdgeEffects _method )
{
    vector<float> temp = gaussianKernel( _sigma );
    CMatrixV<float> Gaus1H( 3,1,temp );
    CMatrixV<float> Gaus1W( 1,3,temp );
    magnitude( m_myImage, convolution( Gaus1W, _method ), convolution( Gaus1H, _method ) );
}

//-----------------------------------------------------------------------------------
vector<int> CImageHandler::resizeBilinear( vector<int>& _image, int _widthOld, int _heightOld, int _widthNew, int _heightNew )
{
    vector<int> temp( _widthNew * _heightNew );
    int a, b, c, d, x, y, index;
    float x_ratio = ( (float)( _widthOld - 1 ) ) / _widthNew;
    float y_ratio = ( (float)( _heightOld - 1 ) ) / _heightNew;
    float x_diff, y_diff;
    int offset = 0;
    for ( auto i = 0; i < _heightNew; i++ )
    {
        for (auto j = 0; j < _widthNew; j++ )
        {
            x = (int)( x_ratio * j );
            y = (int)( y_ratio * i );
            x_diff = ( x_ratio * j ) - x;
            y_diff = ( y_ratio * i ) - y;
            index = ( y  * _widthOld + x );
            a = _image[ index ];
            b = _image[ index + 1 ];
            c = _image[ index + _widthOld ];
            d = _image[ index + _widthOld + 1 ];
            temp[offset++] =  a * ( 1 - x_diff ) * ( 1 - y_diff ) + b * ( x_diff ) * ( 1 - y_diff ) + c * ( y_diff )*( 1 - x_diff ) + d * ( x_diff * y_diff );
        }
    }
    return temp;
}
