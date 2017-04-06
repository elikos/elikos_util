/******************************************************************************
* Classe qui permet de montrer facilement le contenu d'une image a l'écran.
******************************************************************************/
#ifndef REMOTE_CALIB_IMAGE_OUTPUT_H
#define REMOTE_CALIB_IMAGE_OUTPUT_H

#include <QWidget>

#include <rqt_image_view/ratio_layouted_frame.h>

class ImageOutput : public QWidget
{
    Q_OBJECT
public:
    ImageOutput(QWidget* parent);
private:
    rqt_image_view::RatioLayoutedFrame imageFrame_;
};







#endif
