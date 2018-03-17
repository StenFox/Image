#ifndef CIMAGE_H
#define CIMAGE_H
#include <vector>
#include <QImage>
#include "QPixmap"
#include "CMatrixV.h"

enum mtProcessingEdgeEffects
{
    // Считаем что всё что "снаружи" = 0, т.е чёрным
    mtBlackEdge,

    // Копируем значения у границ
    mtCopyEdge,

    // Сворачиваем избражение самого в себя т.е создаём Тор
    mtThor,
};

class CImage
{
public:
    CImage( QString _fileName );
    QImage getImage();
    QImage getOriginalImage();

    // Размытие по Гауссу
    void gaussianBlur( float _sigma, mtProcessingEdgeEffects _method = mtBlackEdge );

    // Оператор Собеля
    void sobel( mtProcessingEdgeEffects _method = mtBlackEdge  );

    // Билинейная интреполяция уменьшаем изображение в 2 раза
    void resizeTwo();

    // Перезагрузка изображения
    void reloadImage();
private:

    // Ядро Собель по X
    static CMatrixV<int> g_sobelX;

    // Ядро Собель по Y
    static CMatrixV<int> g_sobelY;

    // Ширина "моего изображения"
    int m_width;

    // Высота "моего изображения"
    int m_height;

    //Оригинальное изображение
    QImage m_originalImage;

    //Вектор для хранения изображения
    std::vector<int> m_myImage;

    // Функция перевода изображения в оттенки серого
    void grayScale( QImage _image );

    // Магнитуда или вычисление Величины градиента
    void magnitude( std::vector<int>& _input, const std::vector<int>& _gx, const std::vector<int>& _gy );

    // Свёртка, на вход подаём ядро cвёртки и метод для обработки краевых эффектов
    std::vector<int> convolution(CMatrixV<auto> kernel, mtProcessingEdgeEffects _method );

    // Вычисление Гауссиана
    float gaussian( int _x,float _s );

    // Ядро фильтра гаусса
    std::vector<float> gaussianKernel( float _sigma );

    // Две последовательных свёртки для ускорния
    void convolutionForGauss( float _sigma, mtProcessingEdgeEffects _method );

    // Билинейная интерполяция
    std::vector<int> resizeBilinear( std::vector<int> _image, int _widthOld, int heightOld, int _widthNew, int heightNew );
};

#endif // CIMAGE_H
