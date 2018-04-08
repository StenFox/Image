#ifndef CIMAGEHANDLER_H
#define CIMAGEHANDLER_H
#include "CImage.h"
#include "CPyramid.h"
#include "CDescriptor.h"
#include <QDebug>
#include <QPoint>
#include <math.h>

enum mtProcessingEdgeEffects
{
    // Считаем что всё что "снаружи" = 0, т.е чёрным
    mtBlackEdge,

    // Копируем значения у границ
    mtCopyEdge,

    // Сворачиваем избражение самого в себя т.е создаём Тор
    mtThor,
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

    // Билинейная интреполяция уменьшаем изображение в 2 раза
    CImage* resizeTwo( CImage& _myImage );

    // Отобразить точки интереса с помощью Морравика
    QImage showInterestPointMoravec( CImage& _myImage, float T, size_t _windowHeight, size_t _windowWidth );

    // Отобразить точки интереса с помощью Харриса
    QImage showInterestPointHarris( CImage& _myImage, float T, float _k, bool _useNonMaximum, int _colPoints );

private:    
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
                for (auto j = 0; j < kh; j++)
                {
                    if ((y + j < offsety || y + j >= heightImg))
                    {
                        if( _method == mtBlackEdge )
                            continue;
                    }
                    for (auto i = 0; i < kw; i++)
                    {
                        if ( ( x + i < offsetx || x + i >= widthImg ) )
                        {
                            if( _method == mtBlackEdge )
                                continue;
                            if( _method == mtCopyEdge )
                            {
                                sum += _kernel.getItem( j, i ) * _myImage.getPixel( y + j, x + i );
                                continue;
                            }
                            if( _method == mtThor )
                            {
                                if(y + j < offsety )
                                {
                                    if(x + i < offsetx)
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( heightImg, widthImg );
                                    }
                                    else if( x + i >= widthImg )
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( heightImg - 1 , widthImg );
                                    }
                                    else
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( y + j - offsety, x + i - offsetx );
                                    continue;
                                }

                                if( y + j >= heightImg )
                                {
                                    if(x + i < offsetx)
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( 0, widthImg ) ;
                                    }
                                    else if( x + i >= widthImg )
                                    {
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( 0, 0 );
                                    }
                                    else
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( y + j - offsety, x + i - offsetx );
                                    continue;
                                }
                            }
                        }

                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( y + j - offsety , x + i - offsetx );
                    }
                }
                outImage.setPixel( y, x, sum );
            }
        }

        return outImage;
    }

    // Вычисление Гауссиана
    float gaussian( int _x,float _s );

    // Ядро фильтра гаусса
    std::vector<float> gaussianKernel( float _sigma );

    // Две последовательных свёртки для ускорния
    void convolutionForGauss( float _sigma, CImage& _myImage, mtProcessingEdgeEffects _method );

    // Применить свёртку к изображению
    void applyConvolution( std::vector<float>& _vector, CImage& _myImage )
    {
        for (int i = 0; i < _myImage.getHeight(); ++i)
        {
            for (int j = 0; j < _myImage.getWidth(); ++j)
            {
                _myImage.setPixel(i,j,_vector[i*_myImage.getWidth() + j]);
            }
        }
    }

    // Билинейная интерполяция
    std::vector<float> resizeBilinear( const CImage& _myImage, int _widthOld, int _heightOld, int _widthNew, int _heightNew );

    // Отметить на ихображении точки интереса
    QImage setRedPointsOfInterest( CImage& _myImage, std::vector<QPoint> _interestPoints );

    float valueErrorShift( int x, int y, int _sh, size_t _windowHeight, size_t _windowWidth, const CImage& _myImage );

    bool filtrate( int _x, int _y, float _valueOperator, float _T, const CImage& _myImage,int _ambit, int _windowHeight, int _windowWidth, const CImage& _dx, const CImage& _dy, float _k  );
    // Детектор
    std::vector<QPoint> moravec( const CImage& _myImage, float _T, size_t _windowHeight, size_t _windowWidth );

    // Детектор Харриса
    std::vector<QPoint> harris( const CImage& _myImage, float _T , float _k, bool _useNonMaximum, int _colPoint );

    // Подавление не максимальных элементов
    std::vector<QPoint> nonMaximumPoints( std::vector<float>& _value, std::vector<QPoint> _points, int _colPoints );

    float distanceBetweenPoints( QPoint _p1, QPoint _p2 );

    float eigenvaluesHarris( int _x, int _y, const CImage& _dx, const CImage& _dy, float _k, int _ambit  );

    float minErrorShift( int _x, int _y, size_t _windowHeight, size_t _windowWidth, const CImage& _myImage );

    void descriptor(const CImage& _myImage, int _colHistogram, int _colPin, int _ambit, std::vector<QPoint> _interestPoint );
};

#endif // CIMAGEHANDLER_H
