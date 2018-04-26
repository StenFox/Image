#ifndef CIMAGEHANDLER_H
#define CIMAGEHANDLER_H
#include "CImage.h"
#include "CPyramid.h"
#include "CDescriptor.h"
#include <QDebug>
#include <QPoint>
#include <cmath>

enum mtProcessingEdgeEffects
{
    // Считаем что всё что "снаружи" = 0, т.е чёрным
    mtBlackEdge,

    // Копируем значения у границ
    mtCopyEdge,

    // Сворачиваем избражение самого в себя т.е создаём Тор
    mtThor,
};

enum mtHistogramm
{
    mtFourHistgramms,
    mtSixteenHistgramms,
};

class CImageHandler
{
public:

    CImageHandler();

    // Пирамида Гаусса
    CPyramid gaussPyramid( const CImage& _myImage, int _octaves,int sclaes, float sigmaZero );

    // Оператор Собеля
    void sobel( mtProcessingEdgeEffects _method, CImage& _image );

    // Функция перевода изображения в оттенки серого
    void grayScale( QImage& _image, CImage& _myImage );

    // Размытие по Гауссу
    void gaussianBlur( float _sigma, CImage& _myImage, mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Привитта
    void priwitt( mtProcessingEdgeEffects _method, CImage& _image );

    // Оператор Робертса
    void robert( mtProcessingEdgeEffects _method, CImage& _image );

    std::vector<std::pair<CDescriptor,CDescriptor>> imageComparison( CImage& _myImage1, CImage& _myImage2, bool _discardPoints, float _threshold );

    // Детектор Харриса
    std::vector<QPoint> harris( const CImage& _myImage, float _T , float _k,bool _useNonMaximum, int _colPonts );

    // Детектор Моравика
    std::vector<QPoint> moravec( const CImage& _myImage, float _T, size_t _windowHeight, size_t _windowWidth, bool _useNonMaximum, int _colPoints );

    // Вычисляем дескрипторы
    void descriptor( CImage& _myImage, int _colHistogram, int _colPin, int _ambit, const std::vector<QPoint>& _interestPoint );

    // Дескрипторы устойчивые к вращению
    void descriptorRotation( CImage& _myImage, int _ambit, const std::vector<QPoint>& _interestPoint );

    float testDesriptorsForBrightness( CImage& _myImage );

    void brightnessChange( CImage& _myImage, float _value )
    {
        for( int y = 0; y < _myImage.getHeight(); y++ )
        {
            for( int x = 0; x < _myImage.getWidth(); x++ )
            {
                _myImage.setItem( y, x, _myImage.getItem(y,x) + _value );
            }
        }
    }

    void contrastChange( CImage& _myImage, float _contrast )
    {
        float factor = ( 259 * ( _contrast + 255 ) ) / ( 255 * ( 259 - _contrast ) );
        for( int y = 0; y < _myImage.getHeight(); y++ )
        {
            for( int x = 0; x < _myImage.getWidth(); x++ )
            {
                _myImage.setItem( y, x, factor * ( _myImage.getItem( y,x ) - 128 ) + 128 );
            }
        }
    }

private:
    CHistogram m_his;

    // Ядро Собель по X
    static const CMatrixV<int> g_sobelX;

    // Ядро Собель по Y
    static const CMatrixV<int> g_sobelY;

    // Ядро Привитт по X
    static CMatrixV<int> g_prewittX;

    // Ядро Привитт по Y
    static CMatrixV<int> g_prewittY;

    // Ядро Робертс по X
    static CMatrixV<int> g_robertX;

    // Ядро Робертс по Y
    static CMatrixV<int> g_robertY;

    // Вектор сдвигов
    static const std::vector<std::pair<int,int>> g_shiftWindow;

    // Уменьшение изображения в 2 раза с помощью билинейной интерполяции
    void downSpace( CImage& _myImage );

    // Магнитуда или вычисление Величины градиента
    void magnitude(  CImage& _input, const CImage& _gx, const CImage& _gy );

    // Свёртка, на вход подаём ядро cвёртки и метод для обработки краевых эффектов
    template<typename T>
    CImage convolution( const CMatrixV<T>& _kernel, const CImage& _myImage, mtProcessingEdgeEffects _method )
    {
        int kw = _kernel.getRows();
        int kh = _kernel.getColumns();
        auto offsetx = kw / 2;
        auto offsety = kh / 2;
        float sum;

        auto heightImg = _myImage.getHeight();
        auto widthImg = _myImage.getWidth();

        CImage outImage(heightImg,widthImg);
        for (auto y = 0; y < heightImg; y++)
        {
            for (auto x = 0; x < widthImg; x++)
            {
                sum = 0;
                bool flag = false;
                for (auto j = 0; j < kh; j++)
                {
                    if( ( y + j < offsety || y + j >= heightImg ) )
                    {
                        if( _method == mtBlackEdge )
                            continue;
                        flag = true;
                    }
                    for (auto i = 0; i < kw; i++)
                    {
                        if ( ( x + i < offsetx || x + i >= widthImg ) )
                        {
                            if( _method == mtBlackEdge )
                                continue;
                            if( _method == mtCopyEdge )
                            {
                                sum += _kernel.getItem( j, i ) * _myImage.getItem( y, x );
                                continue;
                            }
                            if( _method == mtThor )
                            {
                                if( y + j < offsety )
                                {
                                    if(x + i < offsetx)
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getItem( heightImg, widthImg );
                                    }
                                    else if( x + i >= widthImg )
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getItem( heightImg - 1 , widthImg );
                                    }
                                    else
                                        sum += _kernel.getItem( j, i ) * _myImage.getItem( y + j - offsety, x + i - offsetx );
                                    continue;
                                }

                                if( y + j >= heightImg )
                                {
                                    if(x + i < offsetx)
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getItem( 0, widthImg ) ;
                                    }
                                    else if( x + i >= widthImg )
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getItem( 0, 0 );
                                    }
                                    else
                                        sum += _kernel.getItem( j, i ) * _myImage.getItem( y + j - offsety, x + i - offsetx );
                                    continue;
                                }
                            }
                        }
                        else if( flag )
                        {
                            if( _method == mtCopyEdge )
                            {
                                sum += _kernel.getItem( j, i ) * _myImage.getItem( y, x + i - offsetx );
                                continue;
                            }
                            if( _method == mtThor )
                            {
                                if( y + j < offsety )
                                {
                                    sum += _kernel.getItem( j, i ) * _myImage.getItem(  heightImg , x + i - offsetx );
                                    continue;
                                }

                                if( y + j >= heightImg )
                                {
                                    sum += _kernel.getItem( j, i ) * _myImage.getItem( 0, x + i - offsetx );
                                    continue;
                                }
                            }
                        }

                        sum += _kernel.getItem( j, i ) * _myImage.getItem( y + j - offsety , x + i - offsetx );
                    }
                }
                outImage.setItem( y, x, sum );
            }
        }

        return outImage;
    }

    // Вычисление Гауссиана
    float gaussian( int _x,float _s );

    // Вычисление Гауссиана
    float gaussian( int _x, int _y, float _sigma );

    // Ядро фильтра гаусса
    std::vector<float> gaussianKernel( float _sigma );

    // Две последовательных свёртки для ускорния
    void convolutionForGauss( float _sigma, CImage& _myImage, mtProcessingEdgeEffects _method );

    // Билинейная интерполяция
    std::vector<float> resizeBilinear( const CImage& _myImage, int _widthOld, int _heightOld, int _widthNew, int _heightNew );

    // Отметить на ихображении точки интереса
    QImage setRedPointsOfInterest( CImage& _myImage, std::vector<QPoint> _interestPoints );

    // Вычисляем значение ошибки при сдвиге
    float valueErrorShift( int x, int y, int _sh, size_t _windowHeight, size_t _windowWidth, const CImage& _myImage );

    // Фильтруем точки интереса
    bool filtrate( int _x, int _y, float _valueOperator, float _T, const CImage& _myImage,int _ambit, int _windowHeight, int _windowWidth, const CImage& _dx, const CImage& _dy, float _k  );

    // Подавление не максимальных элементов
    std::vector<QPoint> nonMaximumPoints(  std::vector<float>& _value, std::vector<QPoint>& _points, const int _colPoints  );

    // Растояние между 2 точка
    float distanceBetweenPoints( const QPoint& _p1, const QPoint& _p2 );

    // Собстевенные числа для Харрисона
    float eigenvaluesHarris( int _x, int _y, const CImage& _dx, const CImage& _dy, float _k, int _ambit  );

    // Минимальная ошибка при сдвигах в 8 направлениях
    float minErrorShift( int _x, int _y, size_t _windowHeight, size_t _windowWidth, const CImage& _myImage );

    // Распределяем значения между 4 гистораммами
    void fourHistogramms( const int x, const int y, CDescriptor& _des, float _vG, float _dG );

    // Распределяем значения между 16 гистораммами
    void sixteenHistogramms( const int x, const int y, CDescriptor& _des,float _vG, float _dG, int _ambit );

    // "Расстояние" между дескрипторами
    float distanceBetweenDescriptors( const CDescriptor& _d, const CDescriptor& _d1 );

    // Оринтация точки
    std::vector<float> pointOrientation( const CImage& _direction,const CImage& _value, const QPoint& _point, int _radius );
    float basketIterpolation(const int _index, const std::vector<float>& m_histogramms);

    int compareDes(const std::vector<std::pair< CDescriptor,CDescriptor > >& des);
};

#endif // CIMAGEHANDLER_H
