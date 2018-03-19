#include "CImage.h"
#include "CImageKernels.h"

using namespace  std;

CMatrixV<int> CImage::g_sobelX( 3, 3, VsobelX );
CMatrixV<int> CImage::g_sobelY( 3, 3, VsobelY );

CMatrixV<int> CImage::g_prewittX( 3, 3, VprewittX );
CMatrixV<int> CImage::g_prewittY( 3, 3, VprewittY );

CMatrixV<int> CImage::g_robertX( 3, 3, VrobertX );
CMatrixV<int> CImage::g_robertY( 3, 3, VrobertY );

//-----------------------------------------------------------------------------------
CImage::CImage( QString _fileName )
{
    m_originalImage.load( _fileName );

    m_height = m_originalImage.height();
    m_width = m_originalImage.width();

    m_myImage.reserve( m_height * m_width );

    grayScale( m_originalImage );
}

//-----------------------------------------------------------------------------------
void CImage::grayScale( QImage _image )
{
    Q_ASSERT( _image.format() == QImage::Format_RGB32 );
    for( int i = 0; i < _image.height(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( _image.scanLine( i ) );
        QRgb *end = pixel + _image.width();
        for( ; pixel != end; pixel++ )
        {
            int gray = qGray( *pixel );
            m_myImage.push_back( gray );
            *pixel = QColor( gray, gray, gray ).rgb();
        }
    }
}

//-----------------------------------------------------------------------------------
void CImage::sobel( mtProcessingEdgeEffects _method )
{
    magnitude( m_myImage, convolution( g_sobelX, _method ), convolution( g_sobelY, _method ) );
}

//-----------------------------------------------------------------------------------
void CImage::priwitt( mtProcessingEdgeEffects _method )
{
    magnitude( m_myImage, convolution( g_prewittX, _method ), convolution( g_prewittY, _method ) );
}

//-----------------------------------------------------------------------------------
void CImage::robert( mtProcessingEdgeEffects _method )
{
    magnitude( m_myImage, convolution( g_robertX, _method ), convolution( g_robertY, _method ) );
}

//-----------------------------------------------------------------------------------
void CImage::resizeTwo()
{
    m_myImage = resizeBilinear( m_myImage, m_width, m_height, m_width/2, m_height/2 );
    m_height /= 2;
    m_width /= 2;
}

//-----------------------------------------------------------------------------------
void CImage::magnitude(vector<int>& _input, const vector<int>& _gx, const vector<int>& _gy)
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
vector<int> CImage::convolution(CMatrixV<auto> _kernel, mtProcessingEdgeEffects _method)
{
    int kw = _kernel.getRows();
    int kh = _kernel.getColumns();
    auto offsetx = kw / 2;
    auto offsety = kw / 2;
    float sum;

    vector<int> outConvolution;
    outConvolution.reserve( m_height * m_width );

    for (auto y = 0; y < m_height; y++)
    {
        for (auto x = 0; x < m_width; x++)
        {
            sum = 0;
            for (auto j = 0; j < kh; j++)
            {
                if ((y + j < offsety || y + j >= m_height))
                {
                    if( _method == mtBlackEdge )
                        continue;
                }
                for (auto i = 0; i < kw; i++)
                {
                    if ( (x + i < offsetx || x + i >= m_width) )
                    {
                        if( _method == mtBlackEdge )
                            continue;
                        if( _method == mtCopyEdge )
                        {
                            sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i ];
                            continue;
                        }
                        if( _method == mtThor )
                        {
                            if(y + j < offsety )
                            {
                                if(x + i < offsetx)
                                {
                                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[m_height * m_width];
                                }
                                else if( x + i >= m_width )
                                {
                                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ ( m_height - 1 ) * m_width ];
                                }
                                else
                                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i - offsetx ];
                                continue;
                            }

                            if( y + j >= m_height )
                            {
                                if(x + i < offsetx)
                                {
                                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ m_width] ;
                                }
                                else if( x + i >= m_width )
                                {
                                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ 0 ];
                                }
                                else
                                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i - offsetx ];
                                continue;
                            }
                        }
                    }
                    sum += _kernel.getMatrix()[ j * kw + i ] * m_myImage[ y * m_width + x + i - offsetx ];
                }
            }
            // Нормирование
            outConvolution.push_back( qBound(0x00, static_cast<int>(sum), 0xFF) );
        }
    }

    return outConvolution;
}

//-----------------------------------------------------------------------------------
QImage CImage::getImage()
{
    QImage img ( m_width ,m_height, m_originalImage.format() );
    for ( int i = 0; i < m_height; i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( img.scanLine(i) );
        QRgb *end = pixel + m_width;
        for ( int j =0; pixel != end; pixel++,j++ )
        {
            int gray = m_myImage[ i * m_width + j ];
            *pixel = QColor(gray,gray,gray).rgb();
        }
    }
    return img;
}

//-----------------------------------------------------------------------------------
QImage CImage::getOriginalImage()
{
    return m_originalImage;
}

//-----------------------------------------------------------------------------------
void CImage::gaussianBlur( float _sigma, mtProcessingEdgeEffects _method )
{
    convolutionForGauss( _sigma, _method );
}

//-----------------------------------------------------------------------------------
float CImage::gaussian( int _x,float _s )
{
    return  exp( -( _x * _x ) / _s) / _s / M_PI;
}

//-----------------------------------------------------------------------------------
vector<float> CImage::gaussianKernel( float _sigma )
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
void CImage::convolutionForGauss( float _sigma, mtProcessingEdgeEffects _method )
{
    vector<float> temp = gaussianKernel( _sigma );
    CMatrixV<float> Gaus1H( 3,1,temp );
    CMatrixV<float> Gaus1W( 1,3,temp );
    magnitude( m_myImage, convolution( Gaus1W, _method ), convolution( Gaus1W, _method ) );
    magnitude( m_myImage, convolution( Gaus1H, _method ), convolution( Gaus1H, _method ) );
}

//-----------------------------------------------------------------------------------
vector<int> CImage::resizeBilinear( vector<int> _image, int _widthOld, int _heightOld, int _widthNew, int _heightNew )
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

//-----------------------------------------------------------------------------------
void CImage::reloadImage()
{
    grayScale( m_originalImage );
}
