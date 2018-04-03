#ifndef CIMAGEHANDLER_H
#define CIMAGEHANDLER_H
#include "CImage.h"
#include <QDebug>

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
    void gaussPyramid( CImage& _myImage, int _octaves,int sclaes, float sigmaZero );

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
    void magnitude( CImage& _input, const std::vector<float>& _gx, const std::vector<float>& _gy );

    // Свёртка, на вход подаём ядро cвёртки и метод для обработки краевых эффектов
    template<typename T>
    std::vector<float> convolution(const CMatrixV<T>& _kernel, CImage& _myImage, mtProcessingEdgeEffects _method )
    {
        int kw = _kernel.getRows();
        int kh = _kernel.getColumns();
        auto offsetx = kw / 2;
        auto offsety = kh / 2;
        float sum;

        auto heightImg = _myImage.getHeight();
        auto widthImg = _myImage.getWidth();

        std::vector<float> outConvolution;
        outConvolution.resize( heightImg * widthImg, 0 );
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
                outConvolution[ y * widthImg + x ] = sum;
            }
        }

        return outConvolution;
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
    QImage setRedPointsOfInterest( CImage& _myImage, std::vector<std::pair<int,int>> _interestPoints );

    // Детектор
    std::vector< std::pair<int,int> > moravec( CImage& _myImage, float _T, size_t _windowHeight, size_t _windowWidth );

    // Детектор Харриса
    std::vector< std::pair<int,int> > harris( CImage& _myImage, float _T , float _k, bool _useNonMaximum, int _colPoint );

    // Подавление не максимальных элементов
    std::vector< std::pair<int,int> > nonMaximumPoints( CImage& _myImage, std::vector<float>& _value, int _colPoints );
};

#endif // CIMAGEHANDLER_H
