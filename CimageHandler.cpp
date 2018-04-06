#include "CImageHandler.h"
#include "CImageKernels.h"
#include "CPyramid.h"
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
void CImageHandler::gaussPyramid( CImage& _img, int _octaves,int scales, float sigmaZero )
{
    float sigmaPrev;
    float sigmaNext;
    float deltaSigma;
    sigmaPrev = 0.5;
    float k = pow( 2, (float)1 / scales);
    deltaSigma = pifagor( sigmaPrev,sigmaZero );
    gaussianBlur( deltaSigma,_img, mtBlackEdge );
    for( int i = 0; i < _octaves; i++ )
    {
        sigmaPrev = sigmaZero;
        sigmaNext = sigmaPrev * k;
        deltaSigma = pifagor( sigmaNext,sigmaPrev );
        //while( sigmaNext < 2 * sigmaZero )
        for( int j = 0; j < scales; j++ )
        {
             gaussianBlur( deltaSigma, _img, mtBlackEdge );
             sigmaPrev = sigmaNext;
             sigmaNext = sigmaPrev * k;
        }
        downSpace( _img );
    }
}

//-----------------------------------------------------------------------------------
QImage CImageHandler::setRedPointsOfInterest( CImage& _myImage, vector<pair<int,int>> _interestPoints )
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
         img.setPixel( _interestPoints[i].second, _interestPoints[i].first,red );
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
// T пороговое значиние
vector<pair<int,int>> CImageHandler::moravec( CImage& _myImage, float _T, size_t _windowHeight, size_t _windowWidth  )
{
    auto offsetx = _windowWidth / 2;
    auto offsety = _windowHeight / 2;
    vector<pair<int,int>> point;

    // окрестность
    int p = 3;
    auto offestp = p / 2;
    for( size_t y = 1 + offsety + offestp; y < _myImage.getHeight() - offsety - offestp - 1; y++)
    {
        for( size_t x = 1 + offsetx + offestp; x < _myImage.getWidth() - offsetx - offestp  - 1; x++)
        {
            vector<float> ErrorShift;
            ErrorShift.resize( g_shiftWindow.size() );
            for( size_t sh = 0;sh < g_shiftWindow.size(); sh++ )
            {
                float sum = 0;
                for( size_t j = 0; j < _windowHeight; j++ )
                {
                    for( size_t i = 0; i < _windowWidth; i++ )
                    {
                        float dif = _myImage.getPixel( y + j - offsety,x + i - offsetx ) - _myImage.getPixel( y + j - offsety + g_shiftWindow[sh].first, x + i - offsetx + g_shiftWindow[sh].second );
                        dif = dif * dif;
                        sum += dif;
                    }
                }
                ErrorShift[sh] = sum;
            }
            auto minErrorShift = std::min_element( ErrorShift.begin(),ErrorShift.end() );

            vector<float> ErrorShiftP;
            ErrorShiftP.resize( g_shiftWindow.size() );
            // требование локального максимума
            for( int py = 0 ; py < p; py++ )
            {
                for(int px = 0; px < p; px++ )
                {
                    for( size_t sh = 0;sh < g_shiftWindow.size(); sh++ )
                    {
                        float sum = 0;
                        for( size_t j = 0; j < _windowHeight; j++ )
                        {
                            for( size_t i = 0; i < _windowWidth; i++ )
                            {
                                float dif = _myImage.getPixel( y + py - offestp + j - offsety,x + px - offestp + i - offsetx ) - _myImage.getPixel( y + py - offestp + j - offsety + g_shiftWindow[sh].first, x + px - offestp + i - offsetx + g_shiftWindow[sh].second );
                                dif = dif * dif;
                                sum += dif;
                            }
                        }
                        ErrorShiftP[sh] = sum;
                    }
                }
            }
            auto minErrorShiftP = std::min_element( ErrorShiftP.begin(),ErrorShiftP.end() );

            if( *minErrorShift < *minErrorShiftP )
                continue;
            // требование локального максимума

            if( *minErrorShift > _T )
                point.push_back( make_pair( y,x ) );
        }
    }
    return point;
}

//-----------------------------------------------------------------------------------
vector<pair<int,int>> CImageHandler::harris( CImage& _myImage, float _T , float _k, bool _useNonMaximum, int _colPoints )
{
    auto dx = convolution( g_sobelX, _myImage, mtBlackEdge );
    auto dy = convolution( g_sobelY, _myImage, mtBlackEdge );

    vector<pair<int,int>> point;

    vector<float> value;
    value.resize( _myImage.getHeight() * _myImage.getWidth() );

    float A,B,C,M;

    for( auto j = 0; j < _myImage.getHeight(); j++)
    {
        for( auto i = 0; i < _myImage.getWidth();i++)
        {
            auto dxv = dx.getPixel( j,i );
            auto dyv = dy.getPixel( j,i );
            A = dxv * dxv;
            B = dxv * dyv ;
            C = dyv * dyv;
            M = ( A * C - B * B ) - _k *( ( A + C ) * ( A + C ) );
            if( M > _T )
            {
                point.push_back( make_pair( j,i ) );
                value[ j * _myImage.getWidth() + i ] = M;
            }
        }
    }

    if( _useNonMaximum )
        return nonMaximumPoints( _myImage, value, _colPoints );
    else
        return point;
}

//-----------------------------------------------------------------------------------
vector< pair<int,int> > CImageHandler::nonMaximumPoints( CImage& _myImage ,vector<float>& _value, int _colPoints )
{
    vector<pair<int,int>> point;
    int r = 3;
    auto _col = std::count_if( _value.begin(), _value.end(),[]( float i ){ return i > 0; } );
    while( _col > _colPoints )
    {
        for(int j = 0; j < _myImage.getHeight(); j++)
        {
            for (int i = 0; i < _myImage.getWidth(); i++)
            {
                if( _value[ j * _myImage.getWidth() + i ] == 0 )
                    continue;
                for(int rj = -r; rj <= r; rj++ )
                {
                    for(int ri = -r; ri <= r; ri++ )
                    {
                        int index = j + rj * _myImage.getWidth() + i + ri;
                        if( index < 0 || index >= _myImage.getHeight() * _myImage.getWidth() )
                            continue;
                        if( _value[ j * _myImage.getWidth() + i ] < _value[ j + rj * _myImage.getWidth() + i + ri ] )
                        {
                            _value[ j * _myImage.getWidth() + i ] = 0;
                            break;
                        }
                    }
                }
            }
        }
        _col = std::count_if( _value.begin(),_value.end(),[]( float i ){ return i > 0; } );
        r++;
    }


    for(int j = 0; j < _myImage.getHeight(); j++)
    {
        for( int i = 0; i < _myImage.getWidth(); i++)
        {
            if( _value[ j * _myImage.getWidth() + i ] != 0 )
                point.push_back( make_pair( j,i ) );
        }
    }
    return point;
}

