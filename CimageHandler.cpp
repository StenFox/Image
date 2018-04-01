#include "CImageHandler.h"
#include "CImageKernels.h"

using namespace  std;

const CMatrixV<int> CImageHandler::g_sobelX( 3, 3, VsobelX );
const CMatrixV<int> CImageHandler::g_sobelY( 3, 3, VsobelY );

CMatrixV<int> CImageHandler::g_prewittX( 3, 3, VprewittX );
CMatrixV<int> CImageHandler::g_prewittY( 3, 3, VprewittY );

CMatrixV<int> CImageHandler::g_robertX( 3, 3, VrobertX );
CMatrixV<int> CImageHandler::g_robertY( 3, 3, VrobertY );

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
CImage* CImageHandler::resizeTwo( CImage& _myImg )
{
    vector<float> temp = resizeBilinear( _myImg,_myImg.getWidth(),_myImg.getHeight(),_myImg.getWidth()/2,_myImg.getHeight()/2 );
    return new CImage( _myImg.getHeight()/2,_myImg.getWidth()/2,temp );
}

//-----------------------------------------------------------------------------------
void CImageHandler::downSpace( CImage& _myImg )
{
    int newW =_myImg.getWidth()/2;
    int newH =_myImg.getHeight()/2;
    vector<float> temp = resizeBilinear( _myImg,_myImg.getWidth(),_myImg.getHeight(),newW, newH );
    _myImg.resize(newH,newW,temp);
    temp.clear();
}

//-----------------------------------------------------------------------------------
void CImageHandler::magnitude( CImage& _input, const vector<float>& _gx, const vector<float>& _gy )
{
    for ( auto y = 0; y < _input.getHeight(); y++ )
    {
        for ( auto x = 0; x < _input.getWidth(); x++ )
        {
            _input.setPixel( y, x, hypot ( _gx[ y * _input.getWidth() + x ], _gy[ y * _input.getWidth() + x ] ) );
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
    applyConvolution( g1, myImage );
    auto g2 = convolution( Gaus1H, myImage, _method );
    applyConvolution( g2, myImage );
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

float pifagor(float sigmaNext,float sigmaPrev)
{
    return sqrt(sigmaNext * sigmaNext - sigmaPrev * sigmaPrev);
}

void CImageHandler::gaussPyramid( CImage& _img, int _octaves,int sclaes, float sigmaZero )
{
    float sigmaPrev;
    float sigmaNext;
    float deltaSigma;
    float k = pow( sclaes, (float)1/sclaes);
    sigmaPrev = 0.5;
    sigmaNext = sigmaPrev * k;
    deltaSigma = pifagor( sigmaNext,sigmaPrev );
    while( sigmaNext < sigmaZero )
    {
        gaussianBlur( deltaSigma,_img, mtBlackEdge );
        sigmaPrev = sigmaNext;
        sigmaNext = sigmaPrev * k;
    }

    for (int i = 0; i < _octaves; i++)
    {
        sigmaPrev = sigmaZero;
        sigmaNext = sigmaPrev * k;
        deltaSigma = pifagor( sigmaNext,sigmaPrev );
        while( sigmaNext < 2*sigmaZero )
        {
             gaussianBlur( deltaSigma, _img, mtBlackEdge );
             sigmaPrev = sigmaNext;
             sigmaNext = sigmaPrev * k;
        }
        downSpace( _img );
    }
}
