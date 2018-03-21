#ifndef CIMAGEHANDLER_H
#define CIMAGEHANDLER_H
#include "CImage.h"

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
private:

    // Размытие по Гауссу
    void gaussianBlur( float _sigma, mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Собеля
    void sobel( mtProcessingEdgeEffects _method, CImage& _image );

    // Оператор Привитта
    void priwitt( mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Робертса
    void robert( mtProcessingEdgeEffects _method = mtBlackEdge );

    // Билинейная интреполяция уменьшаем изображение в 2 раза
    void resizeTwo();
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

    // Функция перевода изображения в оттенки серого
    void grayScale( QImage& _image );

    // Магнитуда или вычисление Величины градиента
    void magnitude( CImage& _input, const std::vector<int>& _gx, const std::vector<int>& _gy );

    // Свёртка, на вход подаём ядро cвёртки и метод для обработки краевых эффектов
    template<typename T>
    std::vector<int> convolution(const CMatrixV<T>& _kernel, CImage& _myImage, mtProcessingEdgeEffects _method )
    {
        int kw = _kernel.getRows();
        int kh = _kernel.getColumns();
        auto offsetx = kw / 2;
        auto offsety = kw / 2;
        float sum;

        auto heightImg = _myImage.getHeight();
        auto widthImg = _myImage.getWidth();

        std::vector<int> outConvolution;
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
                                sum += _kernel.getItem( j, i ) * _myImage.getPixel(y, x + i );
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
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( y, x + i - offsetx );
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
                                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( y, x + i - offsetx );
                                    continue;
                                }
                            }
                        }
                        sum += _kernel.getItem( j, i ) * _myImage.getPixel( y, x + i - offsetx );
                    }
                }
                // Нормирование
                outConvolution.push_back( qBound(0x00, static_cast<int>(sum), 0xFF) );
            }
        }

        return outConvolution;
    }

    // Вычисление Гауссиана
    float gaussian( int _x,float _s );

    // Ядро фильтра гаусса
    std::vector<float> gaussianKernel( float _sigma );

    // Две последовательных свёртки для ускорния
    void convolutionForGauss( float _sigma, mtProcessingEdgeEffects _method );

    // Билинейная интерполяция
    std::vector<int> resizeBilinear( const std::vector<int>& _image, int _widthOld, int _heightOld, int _widthNew, int _heightNew );
};

#endif // CIMAGEHANDLER_H
