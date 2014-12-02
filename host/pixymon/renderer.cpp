//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <QPainter>
#include <QFont>
#include "debug.h"
#include <QFile>
#include "renderer.h"
#include "videowidget.h"
#include "interpreter.h"
#include "dataexport.h"
#include "monmodule.h"
#include <chirp.hpp>
#include "calc.h"
#include <math.h>

Renderer::Renderer(VideoWidget *video, Interpreter *interpreter) :
#ifdef DEFER
    m_blobs(interpreter),
#endif
    m_background(0, 0)
{
    m_video = video;
    m_interpreter = interpreter;

    m_rawFrame.m_pixels = new uint8_t[RAWFRAME_SIZE];
    m_rawFrame.m_height = 0;
    m_rawFrame.m_width = 0;

    m_mode = 3;

    connect(this, SIGNAL(image(QImage, uchar)), m_video, SLOT(handleImage(QImage, uchar))); // Qt::BlockingQueuedConnection);
}


Renderer::~Renderer()
{
    delete[] m_rawFrame.m_pixels;
}


inline void Renderer::interpolateBayer(unsigned int width, unsigned int x, unsigned int y, unsigned char *pixel, unsigned int &r, unsigned int &g, unsigned int &b)
{
#if 1
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
            b = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
        }
        else
        {
            r = (*(pixel-1)+*(pixel+1))>>1;
            g = *pixel;
            b = (*(pixel-width)+*(pixel+width))>>1;
        }
    }
    else
    {
        if (x&1)
        {
            r = (*(pixel-width)+*(pixel+width))>>1;
            g = *pixel;
            b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            r = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
            g = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
            b = *pixel;
        }
    }
#endif
#if 0
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = (*(pixel-width-1)+*(pixel-width+1))>>1;
        }
        else
        {
            r = (*(pixel-1)+*(pixel+1))>>1;
            g = *pixel;
            b = *(pixel-width);
        }
    }
    else
    {
        if (x&1)
        {
            r = *(pixel+width);
            g = *pixel;
            b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            r = (*(pixel+width-1)+*(pixel+width+1))>>1;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = *pixel;
        }
    }
#endif
#if 0
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = (*(pixel-width-1)+*(pixel-width+1))>>1;
        }
        else
        {
            r = (*(pixel-1)+*(pixel+1))>>1;
            g = *pixel;
            b = *(pixel-width);
        }
    }
    else
    {
        if (x&1)
        {
            r = *(pixel-width);
            g = *pixel;
            b = (*(pixel-1)+*(pixel+1))>>1;
        }
        else
        {
            r = (*(pixel-width-1)+*(pixel-width+1))>>1;
            g = (*(pixel-1)+*(pixel+1))>>1;
            b = *pixel;
        }
    }
#endif
#if 0
    if (y&1)
    {
        if (x&1)
        {
            r = *pixel;
            g = *(pixel-1);
            b = *(pixel-width-1);
        }
        else
        {
            r = *(pixel-1);
            g = *pixel;
            b = *(pixel-width);
        }
    }
    else
    {
        if (x&1)
        {
            r = *(pixel-width);
            g = *pixel;
            b = *(pixel-1);
        }
        else
        {
            r = *(pixel-width-1);
            g = *(pixel-1);
            b = *pixel;
        }
    }
#endif
}



int Renderer::renderBA81(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
{
    uint16_t x, y;
    uint32_t *line;
    uint32_t r, g, b;

    if (width*height>RAWFRAME_SIZE)
    {
        m_rawFrame.m_width = 0;
        m_rawFrame.m_height = 0;
    }
    else
    {
        memcpy(m_rawFrame.m_pixels, frame, width*height);
        m_rawFrame.m_width = width;
        m_rawFrame.m_height = height;
    }

    // skip first line
    frame += width;

    // don't render top and bottom rows, and left and rightmost columns because of color
    // interpolation
    QImage img(width-2, height-2, QImage::Format_RGB32);

    for (y=1; y<height-1; y++)
    {
        line = (unsigned int *)img.scanLine(y-1);
        frame++;
        for (x=1; x<width-1; x++, frame++)
        {
            interpolateBayer(width, x, y, frame, r, g, b);
            *line++ = (0x40<<24) | (r<<16) | (g<<8) | (b<<0);
        }
        frame++;
    }
    // send image to ourselves across threads
    // from chirp thread to gui thread
    emit image(img, renderFlags);

    m_background = img;

    return 0;
}

void Renderer::renderRects(const Points &points, uint32_t size)
{
    int i;
    QPainter p;
    float scale = (float)m_video->activeWidth()/m_background.width();
    QImage img(m_video->activeWidth(), m_video->activeHeight(), QImage::Format_ARGB32);
    img.fill(0x00000000);

    p.begin(&img);
    p.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x20)));
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));

    DBG("*** size %d", points.size());
    for (i=0; i<points.size(); i++)
        p.drawRect(points[i].m_x*scale, points[i].m_y*scale, size*scale, size*scale);

    p.end();

    emit image(img, RENDER_FLAG_BLEND | RENDER_FLAG_FLUSH);
}

void Renderer::renderRect(const RectA &rect)
{
    QPainter p;
    float scale = (float)m_video->activeWidth()/m_background.width();
    QImage img(m_video->activeWidth(), m_video->activeHeight(), QImage::Format_ARGB32);
    img.fill(0x00000000);

    p.begin(&img);
    p.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x20)));
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));

    p.drawRect(rect.m_xOffset*scale, rect.m_yOffset*scale, rect.m_width*scale, rect.m_height*scale);

    p.end();

    emit image(img, RENDER_FLAG_BLEND | RENDER_FLAG_FLUSH);
}

void Renderer::renderBlobsB(QImage *image, float scale, BlobB *blobs, uint32_t numBlobs)
{
    QPainter p;
    QString str, modelStr;
    uint16_t left, right, top, bottom;
    uint i;

    p.begin(image);
    p.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x20)));
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));

#ifdef __MACOS__
    QFont font("verdana", 18);
#else
    QFont font("verdana", 12);
#endif
    p.setFont(font);
    for (i=0; i<numBlobs; i++)
    {
        left = scale*blobs[i].m_left;
        right = scale*blobs[i].m_right;
        top = scale*blobs[i].m_top;
        bottom = scale*blobs[i].m_bottom;

        //DBG("%d %d %d %d", left, right, top, bottom);
        p.drawRect(left, top, right-left, bottom-top);
        if (blobs[i].m_model)
        {
#if 0
            label = m_blobs.getLabel(model);
            if (label)
                str = *label;
            else
#endif
                modelStr = QString::number(blobs[i].m_model, 8);
            str = "s=" + modelStr + ", " + QChar(0xa6, 0x03) + "=" + QString::number(blobs[i].m_angle);
            p.setPen(QPen(QColor(0, 0, 0, 0xff)));
            p.drawText(left+1, top+1, str);
            p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));
            p.drawText(left, top, str);
        }
    }
    p.end();
}

void Renderer::renderBlobsA(QImage *image, float scale, BlobA *blobs, uint32_t numBlobs)
{
    QPainter p;
    QString str;
    uint16_t left, right, top, bottom;
    uint i;

    p.begin(image);
    p.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x20)));
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));

#ifdef __MACOS__
    QFont font("verdana", 18);
#else
    QFont font("verdana", 12);
#endif
    p.setFont(font);
    for (i=0; i<numBlobs; i++)
    {
        left = scale*blobs[i].m_left;
        right = scale*blobs[i].m_right;
        top = scale*blobs[i].m_top;
        bottom = scale*blobs[i].m_bottom;

        //DBG("%d %d %d %d", left, right, top, bottom);
        p.drawRect(left, top, right-left, bottom-top);
        if (blobs[i].m_model)
        {
#if 0
            label = m_blobs.getLabel(model);
            if (label)
                str = *label;
            else
#endif
                str = str.sprintf("s=%d", blobs[i].m_model);
            p.setPen(QPen(QColor(0, 0, 0, 0xff)));
            p.drawText(left+1, top+1, str);
            p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));
            p.drawText(left, top, str);
        }
    }
    p.end();
}

int Renderer::renderCCB2(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t numBlobs, uint16_t *blobs, uint32_t numCCBlobs, uint16_t *ccBlobs)
{
    float scale = (float)m_video->activeWidth()/width;
    QImage img(width*scale, height*scale, QImage::Format_ARGB32);


    if (renderFlags&RENDER_FLAG_BLEND) // if we're blending, we should be transparent
        img.fill(0x00000000);
    else
        img.fill(0xff000000); // otherwise, we're just black

    numBlobs /= sizeof(BlobA)/sizeof(uint16_t);
    numCCBlobs /= sizeof(BlobB)/sizeof(uint16_t);
    renderBlobsA(&img, scale, (BlobA *)blobs, numBlobs);
    renderBlobsB(&img, scale, (BlobB *)ccBlobs, numCCBlobs);

    emit image(img, renderFlags);

    return 0;
}

int Renderer::renderCCB1(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t numBlobs, uint16_t *blobs)
{
    float scale = (float)m_video->activeWidth()/width;
    QImage img(width*scale, height*scale, QImage::Format_ARGB32);

    if (renderFlags&RENDER_FLAG_BLEND) // if we're blending, we should be transparent
        img.fill(0x00000000);
    else
        img.fill(0xff000000); // otherwise, we're just black

    numBlobs /= sizeof(BlobA)/sizeof(uint16_t);
    renderBlobsA(&img, scale, (BlobA *)blobs, numBlobs);

    emit image(img, renderFlags);

    return 0;
}

void Renderer::renderRL(QImage *image, uint color, uint row, uint startCol, uint len)
{
    uint *line;
    uint col;

    if (row>=(uint)image->height() || startCol>=(uint)image->width() || startCol+len>(uint)image->width())
        return;
    line = (unsigned int *)image->scanLine(row);
    for (col=startCol; col<startCol+len; col++)
        line[col] = color;
}

int Renderer::renderCCQ1(uint8_t renderFlags, uint16_t width, uint16_t height, uint32_t numVals, uint32_t *qVals)
{

    int32_t row;
    uint32_t i, startCol, length;
    uint8_t model;
    QImage img(width, height, QImage::Format_ARGB32);
    unsigned int palette[] =
    {0x00000000, // 0 no model (transparent)
     0x80ff0000, // 1 red
     0x80ff4000, // 2 orange
     0x80ffff00, // 3 yellow
     0x8000ff00, // 4 green
     0x8000ffff, // 5 cyan
     0x800000ff, // 6 blue
     0x80ff00ff  // 7 violet
    };

    // if we're not blending, set alpha to 1.0
    if (!(renderFlags&RENDER_FLAG_BLEND))
    {
        for (i=0; i<sizeof(palette)/sizeof(unsigned int); i++)
            palette[i] |= 0xff000000;
    }

    // q val:
    // | 4 bits    | 7 bits      | 9 bits | 9 bits    | 3 bits |
    // | shift val | shifted sum | length | begin col | model  |

    img.fill(palette[0]);
    for (i=0, row=-1; i<numVals; i++)
    {
        if (qVals[i]==0xffffffff)
            continue;
        if (qVals[i]==0)
        {
            row++;
            continue;
        }
        model = qVals[i]&0x07;
        qVals[i] >>= 3;
        startCol = qVals[i]&0x1ff;
        qVals[i] >>= 9;
        length = qVals[i]&0x1ff;
        renderRL(&img, palette[model], row, startCol, length);
    }

    emit image(img, renderFlags);
    return 0;
}

int Renderer::renderCMV1(uint8_t renderFlags, uint32_t cmodelsLen, float *cmodels, uint16_t width, uint16_t height, uint32_t frameLen, uint8_t *frame)
{
    int i;
    uint32_t numBlobs, numCCBlobs;
    BlobA *blobs;
    BlobB *ccBlobs;
    uint32_t numQvals;
    uint32_t *qVals;

#ifdef DEFER
    if (cmodelsLen>=sizeof(ColorModel)*NUM_MODELS/sizeof(float)) // create lookup table
    {
        m_blobs.m_blobs->m_clut->clear();
        for (i=0; i<NUM_MODELS; i++, cmodels+=sizeof(ColorModel)/sizeof(float))
            m_blobs.m_blobs->m_clut->add((ColorModel *)cmodels, i+1);
    }
#endif
    //m_blobs.process(Frame8(frame, width, height), &numBlobs, &blobs, &numCCBlobs, &ccBlobs, &numQvals, &qVals);

    renderBA81(0, width, height, frameLen, frame);
    renderCCQ1(0, width/2, height/2, numQvals, qVals);
    renderCCB2(RENDER_FLAG_FLUSH, width/2, height/2, numBlobs*sizeof(BlobA)/sizeof(uint16_t), (uint16_t *)blobs, numCCBlobs*sizeof(BlobB)/sizeof(uint16_t), (uint16_t *)ccBlobs);

    return 0;
}

int Renderer::renderBLT1(uint8_t renderFlags, uint16_t width, uint16_t height,
               uint16_t blockWidth, uint16_t blockHeight, uint32_t numPoints, uint16_t *points)
{
    uint i;
    QPainter p;
    float scale = (float)m_video->activeWidth()/width;
    QImage img(m_video->activeWidth(), m_video->activeHeight(), QImage::Format_ARGB32);
    if (renderFlags&RENDER_FLAG_BLEND) // if we're blending, we should be transparent
        img.fill(0x00000000);
    else
        img.fill(0xff000000); // otherwise, we're just black

    p.begin(&img);
    p.setBrush(QBrush(QColor(0xff, 0xff, 0xff, 0x20)));
    p.setPen(QPen(QColor(0xff, 0xff, 0xff, 0xff)));

    for (i=0; i<numPoints; i+=2)
        p.drawRect(points[i]*scale, points[i+1]*scale, blockWidth*scale, blockHeight*scale);

    p.end();

    emit image(img, renderFlags);

    return 0;
}




int Renderer::render(uint32_t type, const void *args[])
{
    int res;
    int i;

    // check modules, see if they handle the fourcc
    for (i=0; i<m_interpreter->m_modules.size(); i++)
    {
        if (m_interpreter->m_modules[i]->render(type, args))
            return 0;
    }

    // choose fourcc for representing formats fourcc.org
    if (type==FOURCC('B','A','8','1'))
        res = renderBA81(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint16_t *)args[2], *(uint32_t *)args[3], (uint8_t *)args[4]);
    else if (type==FOURCC('C','C','Q','1'))
        res = renderCCQ1(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint16_t *)args[2], *(uint32_t *)args[3], (uint32_t *)args[4]);
    else if (type==FOURCC('C', 'C', 'B', '1'))
        res = renderCCB1(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint32_t *)args[2], *(uint32_t *)args[3], (uint16_t *)args[4]);
    else if (type==FOURCC('C', 'C', 'B', '2'))
        res = renderCCB2(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint32_t *)args[2], *(uint32_t *)args[3], (uint16_t *)args[4], *(uint32_t *)args[5], (uint16_t *)args[6]);
//    else if (type==FOURCC('C', 'M', 'V', '1'))
//        res = renderCMV1(*(uint8_t *)args[0], *(uint32_t *)args[1], (float *)args[2], *(uint16_t *)args[3], *(uint32_t *)args[4], *(uint32_t *)args[5], (uint8_t *)args[6]);
    else if (type==FOURCC('B','L','T','1'))
        res = renderBLT1(*(uint8_t *)args[0], *(uint16_t *)args[1], *(uint16_t *)args[2],
                *(uint16_t *)args[3], *(uint16_t *)args[4], *(uint32_t *)args[5], (uint16_t *)args[6]);
    else // format not recognized
        return -1;

    return res;
}


void Renderer::pixelsOut(int x0, int y0, int width, int height)
{
    uint pixel, *line;
    uint r, g, b;
    int x, y;

    DataExport dx(m_interpreter->m_pixymonParameters->value("Document folder").toString(), "pixels", ET_MATLAB);

    dx.startArray(3, "pixels");

    for (y=0; y<height; y++)
    {
        line = (unsigned int *)m_background.scanLine(y0+y);
        for (x=0; x<width; x++)
        {
            pixel = line[x0+x];
            b = pixel&0xff;
            pixel >>= 8;
            g = pixel&0xff;
            pixel >>= 8;
            r = pixel&0xff;

            dx.addElement(r);
            dx.addElement(g);
            dx.addElement(b);
        }
    }
}



int Renderer::renderBackground(uint8_t renderFlags)
{
    if (m_background.width()!=0)
        emit image(m_background, renderFlags);

    return 0;
}

int Renderer::saveImage(const QString &filename)
{
    return m_background.save(filename);
}


QImage *Renderer::backgroundImage()
{
    return &m_background;
}

Frame8 *Renderer::backgroundRaw()
{
    return &m_rawFrame;
}


