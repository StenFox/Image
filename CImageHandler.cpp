#include "CImageHandler.h"
#include "CImageKernels.h"
#include <iostream>
#include <memory>
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
    for( int i = 0; i < _image.height(); i++ )
    {
        QRgb *pixel = reinterpret_cast<QRgb*>( _image.scanLine( i ) );
        QRgb *end = pixel + _image.width();
        for(auto j = 0 ; pixel != end; pixel++,j++ )
        {
            int gray = qGray( *pixel );
            _myImage.setItem(i,j,gray );
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
            _input.setItem( y, x, hypot ( _gx.getItem( y,x ), _gy.getItem( y,x ) ) );
        }
    }
}

//-----------------------------------------------------------------------------------
void CImageHandler::gaussianBlur( float _sigma, CImage& _myImage, mtProcessingEdgeEffects _method )
{
    convolutionForGauss( _sigma, _myImage, _method );
}

//-----------------------------------------------------------------------------------
float CImageHandler::gaussian( int _x, float _sigma )
{
    return  exp( -( _x * _x ) / 2 * _sigma *_sigma ) /(sqrt( 2 * M_PI ) * _sigma );
}

//-----------------------------------------------------------------------------------
float CImageHandler::gaussian( int _x, int _y, float _sigma )
{
    return  exp( -( _x * _x + _y * _y ) / 2 * _sigma *_sigma ) / ( 2 * M_PI  * _sigma * _sigma );
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
            a = _img.getItem( 0,index);
            b = _img.getItem( 0,index+1 );
            c = _img.getItem( 0, index + _widthOld );
            d = _img.getItem( 0, index + _widthOld + 1 );
            temp[offset++] =  a * ( 1 - x_diff ) * ( 1 - y_diff ) + b * ( x_diff ) * ( 1 - y_diff ) + c * ( y_diff )*( 1 - x_diff ) + d * ( x_diff * y_diff );
        }
    }
    return temp;
}

//-----------------------------------------------------------------------------------
float pythagoras( float sigmaNext, float sigmaPrev )
{
    return sqrt( sigmaNext * sigmaNext - sigmaPrev * sigmaPrev );
}

//-----------------------------------------------------------------------------------
CPyramid CImageHandler::gaussPyramid( const CImage& _img, int _octaves,int _scales, float sigmaZero )
{
    float sigmaPrev;
    float sigmaNext;
    float deltaSigma;
    sigmaPrev = 0.5;
    float k = pow( 2, (float)1 / _scales);
    deltaSigma = pythagoras( sigmaZero,sigmaPrev );
    CImage temp = _img;
    gaussianBlur( deltaSigma, temp , mtBlackEdge );

    CPyramid pyramid( _octaves + 1,sigmaZero, _scales );
    pyramid.setImageInOctaves( temp,0,deltaSigma );
    for( int i = 1; i <= _octaves; i++ )
    {
        sigmaPrev = sigmaZero;
        sigmaNext = sigmaPrev * k;
        deltaSigma = pythagoras( sigmaNext,sigmaPrev );
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
vector<QPoint> CImageHandler::moravec(const CImage& _myImage, float _T, size_t _windowHeight, size_t _windowWidth, bool _useNonMaximum, int _colPoints  )
{
    auto offsetx = _windowWidth / 2;
    auto offsety = _windowHeight / 2;
    vector<QPoint> point;
    vector<float> values;

    int p = 3;
    auto offestp = p / 2;

    for( size_t y = 1 + offsety + offestp; y < _myImage.getHeight() - offsety - offestp - 1; y++)
    {
        for( size_t x = 1 + offsetx + offestp; x < _myImage.getWidth() - offsetx - offestp  - 1; x++)
        {
            float minError = minErrorShift( x, y, _windowHeight, _windowWidth, _myImage );
            if( filtrate( x, y, minError, _T, _myImage, p, _windowHeight, _windowWidth, _myImage, _myImage, 0 ) )
            {
                point.push_back( QPoint( x, y ) );
                values.push_back( minError );
            }

        }
    }

    if(_useNonMaximum)
        nonMaximumPoints( values, point, _colPoints );
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
            float dif = _myImage.getItem( _y + j - offsety, _x + i - offsetx ) - _myImage.getItem( _y + j - offsety + g_shiftWindow[_sh].first, _x + i - offsetx + g_shiftWindow[_sh].second );
            dif = dif * dif;
            sum += dif;
        }
    }

    return sum;
}

//-----------------------------------------------------------------------------------
bool CImageHandler::filtrate( int _x, int _y, float _valueOperator, float _T, const CImage& _myImage, int _ambit, int _windowHeight, int _windowWidth, const CImage& _dx, const CImage& _dy, float _k  )
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
            if( _windowHeight != 0 && _windowWidth != 0 )
            {
                valueOperatorInAmbit = minErrorShift( _x + px - offestAmbit, _y + py - offestAmbit, _windowHeight, _windowWidth, _myImage );
            }
            else if( _k != 0 )
            {
                valueOperatorInAmbit = eigenvaluesHarris( _x + px - offestAmbit, _y + py - offestAmbit, _dx, _dy, _k, _ambit );
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
    auto offsetp = p/2;

    for( auto y = 1 + offsetp ; y < _myImage.getHeight() - 1 - offsetp; y++ )
    {
        for( auto x = 1 + offsetp; x < _myImage.getWidth() - 1 - offsetp; x++ )
        {
            float M = eigenvaluesHarris( x, y, dx, dy, _k, p );
            if( filtrate( x, y, M, _T, _myImage, p , 0 , 0, dx, dy, _k ) )
            {
                value.push_back( M );
                point.push_back( QPoint( x,y ) );
            }
        }
    }

    if( _useNonMaximum )
        nonMaximumPoints( value,point,_colPoints );

    return point;
}

//-----------------------------------------------------------------------------------
float CImageHandler::eigenvaluesHarris( const int _x, const int _y, const CImage& _dx, const CImage& _dy, const float _k, const int _ambit  )
{
    float A = 0;
    float B = 0;
    float C = 0;

    auto offestp = _ambit / 2;

    for( int py = 0; py < _ambit; py++ )
    {
        for( int px = 0; px < _ambit; px++ )
        {
           float dxv = _dx.getItem( _y + py - offestp, _x + px - offestp );
           float dyv = _dy.getItem( _y + py - offestp, _x + px - offestp );
           A += dxv * dxv;
           B += dxv * dyv;
           C += dyv * dyv;
        }
    }
    return ( A * C - B * B ) - _k *( ( A + C ) * ( A + C ) );
}

//-----------------------------------------------------------------------------------
float CImageHandler::distanceBetweenPoints( const QPoint& _p1, const QPoint& _p2 )
{
    return sqrt( ( ( _p1.x() - _p2.x() ) * ( _p1.x() - _p2.x() ) ) + ( ( _p1.y() - _p2.y() ) * ( _p1.y() - _p2.y() ) ) );
}

//-----------------------------------------------------------------------------------
vector<QPoint> CImageHandler::nonMaximumPoints(  vector<float>& _value, vector<QPoint>& _points, int _colPoints  )
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

//-----------------------------------------------------------------------------------
void CImageHandler::descriptor( CImage& _myImage, int _colHistogram, int _colBasket, const int _ambit, const vector<QPoint>& _interestPoint )
{
    vector<CDescriptor> descriptors;
    descriptors.resize( _interestPoint.size(), CDescriptor( _colBasket, 16 ) );

    // Предварительные вычисления
    auto dx = convolution( g_sobelX, _myImage, mtBlackEdge );
    auto dy = convolution( g_sobelY, _myImage, mtBlackEdge );

    CImage valueGradient( _myImage.getHeight(),_myImage.getWidth() );
    magnitude( valueGradient, dx, dy );

    CImage directionGradient( _myImage.getHeight(),_myImage.getWidth() );

    for( auto y = 0; y < directionGradient.getHeight(); y++ )
    {
        for( auto x = 0; x < directionGradient.getWidth(); x++ )
        {
            double phi = ( atan2( dx.getItem( y, x ), dy.getItem( y, x ) ) ) + M_PI;
            directionGradient.setItem( y, x, phi );
        }
    }
    // Предварительные вычисления

    int radius = _ambit / 2;
    for( size_t  k = 0; k < _interestPoint.size(); k++ )
    {
        descriptors[k].setInterestPoint( _interestPoint[k] );
        for( int y = -radius; y < radius; y++ )
        {
            for( int x = -radius; x < radius; x++ )
            {
                int yP = _interestPoint[k].y() + y;
                int xP = _interestPoint[k].x() + x;
                if( _myImage.isValid(  yP,xP ) )
                {
                    float vG = valueGradient.getItem( yP,xP );
                    float dG = directionGradient.getItem( yP,xP );
                    sixteenHistogramms( x, y, descriptors[k], vG, dG );
                }
            }
        }
        descriptors[k].normalize( 0.2 );
    }
    _myImage.setDesriptors( descriptors );
}

//-----------------------------------------------------------------------------------
void CImageHandler::fourHistogramms( const int x, const int y, CDescriptor& _des, const float _vG, const float _dG )
{
    int gist = 0;
    if( x < 0 && y < 0 )
        gist = 2;
    if( x < 0 && y >= 0 )
        gist = 0;
    if( x >= 0 && y < 0 )
        gist = 3;
    if( x >= 0 && y >= 0 )
        gist = 1;
    _des.addValueInHistogramm( _vG, _dG, gist );
}

//-----------------------------------------------------------------------------------
void CImageHandler::sixteenHistogramms( const int x, const int y, CDescriptor& _des, const float _vG, const float _dG )
{
    int gist = 0;
    if( x <= 3 && y <= 3 )
        gist = 0;
    else if( x <= 7 && y <= 3 )
        gist = 1;
    else if( x <= 11 && y <= 3 )
        gist = 2;
    else if( x <= 15 && y <= 3 )
        gist = 3;
    else if( x <= 3 && y <= 7 )
        gist = 4;
    else if( x <= 7 && y <= 7 )
        gist = 5;
    else if( x <= 11 && y <= 7 )
        gist = 6;
    else if( x <= 15 && y <= 7 )
        gist = 7;
    else if( x <= 3 && y <= 11 )
        gist = 8;
    else if( x <= 7 && y <= 11 )
        gist = 9;
    else if( x <= 11 && y <= 11 )
        gist = 10;
    else if( x <= 15 && y <= 11 )
        gist = 11;
    else if( x <= 3 && y <= 15 )
        gist = 12;
    else if( x <= 7 && y <= 15 )
        gist = 13;
    else if( x <= 11 && y <= 15 )
        gist = 14;
    else if( x <= 15 && y <= 15 )
        gist = 15;
    _des.addValueInHistogramm( _vG, _dG, gist );
}

//-----------------------------------------------------------------------------------
float CImageHandler::distanceBetweenDescriptors( const CDescriptor& _d1, const CDescriptor& _d2 )
{
    Q_ASSERT( _d1.getColHistogramms() == _d2.getColHistogramms() );
    Q_ASSERT( _d1.getHistograms(0).getColBasket() == _d2.getHistograms(0).getColBasket() );

    float Edistance = 0;
    for( int i = 0; i < _d1.getColHistogramms(); i++)
    {
        int colBasket = _d1.getHistograms(i).getColBasket();
        for( int j = 0; j < colBasket; j++ )
        {
            float difference;
            difference = _d1.getHistograms(i).getValueinBasket(j) - _d2.getHistograms(i).getValueinBasket( j );
            difference *= difference;
            Edistance += difference;
        }
    }
    Edistance = sqrt( Edistance );
    return Edistance;
}

//-----------------------------------------------------------------------------------
vector<pair<CDescriptor,CDescriptor>> CImageHandler::imageComparison( CImage& _myImage1, CImage& _myImage2, bool _discardPoints, float _threshold )
{
    //descriptor( _myImage1, 16, 8, 16, moravec( _myImage1, 5000, 3, 3, false, 0 ) );
    //descriptor( _myImage2, 16, 8, 16, moravec( _myImage2, 5000, 3, 3, false, 0 ) );

    //descriptorRotation( _myImage1, 16, moravec( _myImage1, 5000, 3, 3, false, 0 ) );
    //descriptorRotation( _myImage2, 16, moravec( _myImage2, 5000, 3, 3, false, 0 ) );

    auto des1 = _myImage1.getDescriptors();
    auto des2 = _myImage2.getDescriptors();

    vector<pair<CDescriptor,CDescriptor>> sop;
    for( size_t i = 0; i < des1.size(); i++ )
    {
       float min = std::numeric_limits<float>::max();
       float min2 = min;
       int jmin = 0;
       for( size_t j = 0; j < des2.size(); j++ )
       {
          float Edistance = distanceBetweenDescriptors( des1[i], des2[j] );
          if( Edistance < min )
          {
              jmin = j;
              min = Edistance;
          }
          if( min2 > Edistance && Edistance != min && min2 > min  )
          {
              min2 = Edistance;
          }
          if( min == 0 )
              break;
       }
       if( min / min2 > _threshold )
           continue;
       if( _discardPoints )
       {
           sop.push_back( make_pair( des1[i], des2[jmin]) );
           des1.erase( des1.begin() + i );
           des2.erase( des2.begin() + jmin );
           i--;
       }
       else
       {
           sop.push_back( make_pair( des1[i], des2[jmin]) );
       }
    }
    return sop;
}


//-----------------------------------------------------------------------------------
void CImageHandler::descriptorRotation( CImage& _myImage, int _ambit, const vector<QPoint>& _interestPoint )
{
    vector<CDescriptor> descriptors;
    descriptors.resize( _interestPoint.size(), CDescriptor( 8,16 ) );

    auto dx = convolution( g_sobelX, _myImage, mtBlackEdge );
    auto dy = convolution( g_sobelY, _myImage, mtBlackEdge );

    CImage valueGradient( _myImage.getHeight(),_myImage.getWidth() );
    magnitude( valueGradient, dx, dy );

    CImage directionGradient( _myImage.getHeight(),_myImage.getWidth() );

    for( auto y = 0; y < directionGradient.getHeight(); y++ )
    {
        for( auto x = 0; x < directionGradient.getWidth(); x++ )
        {
            double phi = ( atan2( dx.getItem( y, x ), dy.getItem( y, x ) ) ) + M_PI;
            directionGradient.setItem( y, x, phi );
        }
    }

    int radius = _ambit / 2;
    for( size_t  k = 0; k < _interestPoint.size(); k++ )
    {
        descriptors[k].setInterestPoint( _interestPoint[k] );
        auto peaks = pointOrientation( directionGradient, valueGradient, _interestPoint[k], radius );
        for( size_t i = 0; i < peaks.size(); i++ )
        {
            for( int y = -radius; y < radius; y++ )
            {
                for( int x = -radius; x < radius; x++ )
                {
                    int yP = _interestPoint[k].y() + y;
                    int xP = _interestPoint[k].x() + x;
                    if( _myImage.isValid(  yP,xP ) )
                    {
                        float vG = valueGradient.getItem( yP,xP );
                        float dG = directionGradient.getItem( yP,xP ) + 2 * M_PI -  peaks[i];
                        dG = fmod( dG, 2 * M_PI );
                        int y_Rotate = round( (x) * cos( peaks[i] ) + y * sin( peaks[i] ) );
                        int x_Rotate = round( (y) * cos( peaks[i] ) - x * sin( peaks[i] ) );
                        if( x_Rotate < -radius || x_Rotate >= radius || y_Rotate < -radius || y_Rotate >= radius  )
                            continue;
                        sixteenHistogramms( x_Rotate, y_Rotate, descriptors[k], vG, dG );
                    }
                }
            }
        }
        descriptors[k].normalize( 0.2 );
    }
    _myImage.setDesriptors( descriptors );
}

//-----------------------------------------------------------------------------------
vector<float> CImageHandler::pointOrientation( const CImage& _direction,const CImage& _value, const QPoint& _point, int _radius )
{
    const float sigma = 15;
    // Здесь происходит порча памяти, никак не могу понять как это связанно с ~CHistogram()
    // Heap block at 19913FE8 modified at 19914080 past requested size of 90
    // Работает всё верно, но что-то здесь не то
    m_his.setColBasket( 36 );
    for( int y = -_radius; y < _radius; y++ )
    {
        for( int x = -_radius; x < _radius; x++ )
        {
            int yP = _point.y() + y;
            int xP = _point.x() + x;
            if( _direction.isValid( yP, xP ) && _value.isValid( yP, xP ) )
            {
                float vG = _value.getItem( yP,xP ) * gaussian( x, y, 2 * sigma );
                float dG = _direction.getItem( yP,xP );
                m_his.addValueinBasket( vG, dG );
            }
        }
    }
    return m_his.getPeaks();
}
