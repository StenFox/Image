#include "CImageHandler.h"
#include "CImageKernels.h"
#include <map>

using namespace  std;

const CMatrixV<int> CImageHandler::g_sobelX( 3, 3, VsobelX );
const CMatrixV<int> CImageHandler::g_sobelY( 3, 3, VsobelY );

CMatrixV<int> CImageHandler::g_prewittX( 3, 3, VprewittX );
CMatrixV<int> CImageHandler::g_prewittY( 3, 3, VprewittY );

CMatrixV<int> CImageHandler::g_robertX( 3, 3, VrobertX );
CMatrixV<int> CImageHandler::g_robertY( 3, 3, VrobertY );

const vector<pair<int,int>> CImageHandler::g_shiftWindow = { {1,0}, {-1,0}, {0,1}, {0,-1}, {1,1}, {1,-1}, {-1,1} , {-1,-1} };

//-----------------------------------------------------------------------------------
CImageHandler::CImageHandler()
{

}

//-----------------------------------------------------------------------------------
void CImageHandler::grayScale( QImage& _image, CImage& _myImage )
{
    Q_ASSERT( _image.format() == QImage::Format_RGB32 );
    for( int i = 0; i < _image.height(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( _image.scanLine( i ) );
        QRgb *end = pixel + _image.width();
        for(auto j = 0 ; pixel != end; pixel++,j++ )
        {
            int gray = qGray( *pixel );
            _myImage.setPixel(i,j,gray );
        }
    }
}

//-----------------------------------------------------------------------------------
void CImageHandler::sobel( mtProcessingEdgeEffects _method, CImage& _image )
{
    magnitude( _image, convolution( g_sobelX, _image, _method ), convolution( g_sobelY, _image, _method ) );
}

//-----------------------------------------------------------------------------------
void CImageHandler::priwitt( mtProcessingEdgeEffects _method, CImage& _image )
{
    magnitude( _image, convolution( g_prewittX, _image, _method ), convolution( g_prewittY, _image , _method ) );
}

//-----------------------------------------------------------------------------------
void CImageHandler::robert( mtProcessingEdgeEffects _method, CImage& _image  )
{
    magnitude( _image, convolution( g_robertX, _image, _method ), convolution( g_robertY, _image, _method ) );
}

//-----------------------------------------------------------------------------------
CImage* CImageHandler::resizeTwo( CImage& _myImage )
{
    vector<float> temp = resizeBilinear( _myImage,_myImage.getWidth(),_myImage.getHeight(),_myImage.getWidth()/2,_myImage.getHeight()/2 );
    return new CImage( _myImage.getHeight()/2,_myImage.getWidth()/2,temp );
}

//-----------------------------------------------------------------------------------
void CImageHandler::downSpace( CImage& _myImage )
{
    int newW =_myImage.getWidth()/2;
    int newH =_myImage.getHeight()/2;
    vector<float> temp = resizeBilinear( _myImage,_myImage.getWidth(),_myImage.getHeight(),newW, newH );
    _myImage.resize(newH,newW,temp);
    temp.clear();
}

//-----------------------------------------------------------------------------------
void CImageHandler::magnitude( CImage& _input, const CImage& _gx, const CImage& _gy )
{
    for ( auto y = 0; y < _input.getHeight(); y++ )
    {
        for ( auto x = 0; x < _input.getWidth(); x++ )
        {
            _input.setPixel( y, x, hypot ( _gx.getPixel( y,x ), _gy.getPixel( y,x ) ) );
        }
    }
}

//-----------------------------------------------------------------------------------
void CImageHandler::gaussianBlur( float _sigma,CImage& _myImage , mtProcessingEdgeEffects _method )
{
    convolutionForGauss( _sigma, _myImage, _method );
}

//-----------------------------------------------------------------------------------
float CImageHandler::gaussian( int _x,float _sigma )
{
    return  exp( -( _x * _x ) / 2 * _sigma *_sigma ) /(sqrt( 2 * M_PI ) * _sigma );
}

//-----------------------------------------------------------------------------------
vector<float> CImageHandler::gaussianKernel( float _sigma )
{
    unsigned sizeKernel = 3 * _sigma * 2;

    // ядро меньше 2 не имеет смысла
    if( sizeKernel < 1 )
        sizeKernel = 3;

    // обеспечиваем нечётность размерности ядра
    if( sizeKernel % 2 == 0 )
        sizeKernel++;

    // Резервируем память
    vector<float> gaussKernel1D;

    gaussKernel1D.resize( sizeKernel,0 );
    int edgeKernel = sizeKernel/2;

    float sum = 0;
    //float s = 2 * _sigma * _sigma;

    for (int i = 0, x = -edgeKernel ; x <= edgeKernel; x++,i++ )
    {
        auto temp = gaussian( x,_sigma );
        gaussKernel1D[i]= temp;
        sum += temp;
    }

    for ( size_t i = 0; i < gaussKernel1D.size(); i++ )
    {
        gaussKernel1D[i] /= sum;
    }

    return gaussKernel1D;
}

//-----------------------------------------------------------------------------------
void CImageHandler::convolutionForGauss( float _sigma, CImage& myImage ,mtProcessingEdgeEffects _method )
{
    vector<float> temp = gaussianKernel( _sigma );
    const CMatrixV<float> Gaus1H( temp.size(),1,temp );
    const CMatrixV<float> Gaus1W( 1,temp.size(),temp );
    auto g1 = convolution( Gaus1W, myImage, _method );
    auto g2 = convolution( Gaus1H, g1, _method );
    myImage = std::move( g2 );
}

//-----------------------------------------------------------------------------------
vector<float> CImageHandler::resizeBilinear( const CImage& _img, int _widthOld, int _heightOld, int _widthNew, int _heightNew )
{
    vector<float> temp;
    temp.resize( _widthNew * _heightNew );
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
            a = _img.getPixel( 0,index);
            b = _img.getPixel( 0,index+1 );
            c = _img.getPixel( 0, index + _widthOld );
            d = _img.getPixel( 0, index + _widthOld + 1 );
            temp[offset++] =  a * ( 1 - x_diff ) * ( 1 - y_diff ) + b * ( x_diff ) * ( 1 - y_diff ) + c * ( y_diff )*( 1 - x_diff ) + d * ( x_diff * y_diff );
        }
    }
    return temp;
}

//-----------------------------------------------------------------------------------
float pifagor(float sigmaNext,float sigmaPrev)
{
    return sqrt(sigmaNext * sigmaNext - sigmaPrev * sigmaPrev);
}

//-----------------------------------------------------------------------------------
CPyramid CImageHandler::gaussPyramid( const CImage& _img, int _octaves,int _scales, float sigmaZero )
{
    float sigmaPrev;
    float sigmaNext;
    float deltaSigma;
    sigmaPrev = 0.5;
    float k = pow( 2, (float)1 / _scales);
    deltaSigma = pifagor( sigmaPrev,sigmaZero );
    CImage temp = _img;
    gaussianBlur( deltaSigma, temp , mtBlackEdge );

    CPyramid pyramid( _octaves,sigmaZero, _scales );
    pyramid.setImageInOctaves( temp,0,deltaSigma );
    for( int i = 1; i < _octaves; i++ )
    {
        sigmaPrev = sigmaZero;
        sigmaNext = sigmaPrev * k;
        deltaSigma = pifagor( sigmaNext,sigmaPrev );
        for( int j = 0; j < _scales; j++ )
        {
             gaussianBlur( deltaSigma, temp, mtBlackEdge );
             sigmaPrev = sigmaNext;
             sigmaNext = sigmaPrev * k;
        }
        downSpace( temp );
        pyramid.setImageInOctaves( temp,i,deltaSigma );
    }
    return pyramid;
}

//-----------------------------------------------------------------------------------
QImage CImageHandler::setRedPointsOfInterest( CImage& _myImage, vector<QPoint> _interestPoints )
{
    QImage img ( _myImage.getWidth() ,_myImage.getHeight(), QImage::Format_RGB32 );
    _myImage.normalizeImage();
    for ( int i = 0; i < _myImage.getHeight(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( img.scanLine( i ) );
        QRgb *end = pixel + _myImage.getHeight();
        for ( int j =0; pixel != end; pixel++,j++ )
        {            
             int item = _myImage.getPixel( i,j );
             *pixel = QColor( item, item, item ).rgb();
        }
    }

    auto red = QColor( 255, 0, 0 ).rgb();
    for( size_t i = 0; i < _interestPoints.size(); i++)
    {
         img.setPixel( _interestPoints[i].x(), _interestPoints[i].y(), red );
    }

    return img;
}

//-----------------------------------------------------------------------------------
QImage CImageHandler::showInterestPointMoravec( CImage& _myImage, float T, size_t _windowHeight, size_t _windowWidth )
{
    return setRedPointsOfInterest( _myImage, moravec( _myImage,T, _windowHeight, _windowWidth ) );
}

//-----------------------------------------------------------------------------------
QImage CImageHandler::showInterestPointHarris( CImage& _myImage, float T, float _k, bool _useNonMaximum, int _colPoints )
{
    return setRedPointsOfInterest( _myImage, harris( _myImage,T, _k, _useNonMaximum, _colPoints ) );
}

//-----------------------------------------------------------------------------------
vector<QPoint> CImageHandler::moravec(const CImage& _myImage, float _T, size_t _windowHeight, size_t _windowWidth  )
{
    auto offsetx = _windowWidth / 2;
    auto offsety = _windowHeight / 2;
    vector<QPoint> point;

    int p = 3;
    auto offestp = p / 2;

    for( size_t y = 1 + offsety + offestp; y < _myImage.getHeight() - offsety - offestp - 1; y++)
    {
        for( size_t x = 1 + offsetx + offestp; x < _myImage.getWidth() - offsetx - offestp  - 1; x++)
        {
            float minError = minErrorShift( x, y, _windowHeight, _windowWidth, _myImage );
            if( filtrate( x, y, minError, _T, _myImage, p, _windowHeight, _windowWidth ) )
                point.push_back( QPoint( x, y ) );

        }
    }
    return point;
}

//-----------------------------------------------------------------------------------
float CImageHandler::minErrorShift( int _x, int _y, size_t _windowHeight, size_t _windowWidth, const CImage& _myImage )
{
    vector<float> ErrorShift;
    ErrorShift.resize( g_shiftWindow.size() );
    for( size_t sh = 0;sh < g_shiftWindow.size(); sh++ )
    {
        ErrorShift[sh] = valueErrorShift( _x, _y, sh, _windowHeight, _windowWidth, _myImage );
    }
    auto minErrorShift = std::min_element( ErrorShift.begin(),ErrorShift.end() );
    return *minErrorShift;
}

//-----------------------------------------------------------------------------------
float CImageHandler::valueErrorShift( int _x, int _y, int _sh, size_t _windowHeight, size_t _windowWidth, const CImage& _myImage )
{
    auto offsetx = _windowWidth / 2;
    auto offsety = _windowHeight / 2;
    float sum = 0;

    for( size_t j = 0; j < _windowHeight; j++ )
    {
        for( size_t i = 0; i < _windowWidth; i++ )
        {
            float dif = _myImage.getPixel( _y + j - offsety, _x + i - offsetx ) - _myImage.getPixel( _y + j - offsety + g_shiftWindow[_sh].first, _x + i - offsetx + g_shiftWindow[_sh].second );
            dif = dif * dif;
            sum += dif;
        }
    }

    return sum;
}

//-----------------------------------------------------------------------------------
bool CImageHandler::filtrate( int _x, int _y, float _valueOperator, float _T, const CImage& _myImage,int _ambit, int _windowHeight, int _windowWidth )
{
    // требование порогового значение
    if( _valueOperator < _T )
        return false;

    // окрестность
    auto offestAmbit = _ambit / 2;
    // требование локального максимума
    for( int py = 0 ; py < _ambit; py++ )
    {
        for( int px = 0; px < _ambit; px++ )
        {
            float valueOperatorInAmbit = 0;
            if(true)
            {
                valueOperatorInAmbit = minErrorShift( _x + px - offestAmbit, _y + py - offestAmbit, _windowHeight, _windowWidth, _myImage );
            }
            else
            {

            }
            if( _valueOperator < valueOperatorInAmbit )
                return false;
        }
    }
    return true;
}

//-----------------------------------------------------------------------------------
vector<QPoint> CImageHandler::harris( const CImage& _myImage, float _T , float _k, bool _useNonMaximum, int _colPoints )
{
    auto dx = convolution( g_sobelX, _myImage, mtBlackEdge );
    auto dy = convolution( g_sobelY, _myImage, mtBlackEdge );

    vector<QPoint> point;

    vector<float> value;
    size_t p = 3;

    for( int j = 0; j < _myImage.getHeight(); j++)
    {
        for( int i = 0; i < _myImage.getWidth();i++)
        {
            float M = eigenvaluesHarris( i, j, dx, dy, _k, p );
            if( filtrate( i, j, M, _T, _myImage, p , 0 , 0 ) )
            {
                value.push_back( M );
                point.push_back( QPoint( i,j ) );
            }
        }
    }

    return point;
}

//-----------------------------------------------------------------------------------
float CImageHandler::eigenvaluesHarris( int _x, int _y, const CImage& _dx, const CImage& _dy, float _k, int _ambit  )
{
    float A = 0;
    float B = 0;
    float C = 0;

    auto offestp = _ambit / 2;

    for( int py = 0; py < _ambit; py++ )
    {
        for( int px = 0; px < _ambit; px++ )
        {
           auto dxv = _dx.getPixel( _y + py - offestp,_x + px - offestp );
           auto dyv = _dy.getPixel( _y + py - offestp,_x + px - offestp );
           A += dxv * dxv;
           B += dxv * dyv ;
           C += dyv * dyv;
        }
    }
    return ( A * C - B * B ) - _k *( ( A + C ) * ( A + C ) );
}

//-----------------------------------------------------------------------------------
float CImageHandler::distanceBetweenPoints( QPoint _p1, QPoint _p2 )
{
    return sqrt( ( ( _p1.x() - _p2.x() ) * ( _p1.x() - _p2.x() ) ) + ( ( _p1.y() - _p2.y() ) * ( _p1.y() - _p2.y() ) ) );
}

//-----------------------------------------------------------------------------------
vector<QPoint> CImageHandler::nonMaximumPoints( vector<float>& _value, vector<QPoint> _points, int _colPoints  )
{
    int r = 3;
    while( _points.size() > _colPoints )
    {
        for( size_t i = 0 ; i < _points.size(); i++ )
        {
            for(size_t j = i + 1; j < _points.size(); j++ )
            {
                if( distanceBetweenPoints( _points[i], _points[j] ) < r )
                {
                    if( _value[i] < _value[j] )
                    {
                        _value.erase( _value.begin() + i );
                        _points.erase( _points.begin() + i);
                        i--;
                    }
                }
            }
        }
        r++;
    }
    return _points;
}

