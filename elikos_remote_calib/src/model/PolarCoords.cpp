#include "elikos_remote_calib/model/PolarCoords.h"

/******************************************************************************
* Constructeur par défaut. Tout à 0.
******************************************************************************/
ValeurPolaire::ValeurPolaire()
    : cosinus(0)
    , sinus(0)
{
}

/******************************************************************************
* Initialise une valeur polaire avec un cosinus et un sinus.
******************************************************************************/
ValeurPolaire::ValeurPolaire(double cos, double sin)
    : cosinus(cos)
    , sinus(sin)
{
}








/******************************************************************************
* Initialise une table de coordoonés polaires.
******************************************************************************/
PolarCoordTable::PolarCoordTable()
{
    for (unsigned i = 0; i < MAX_ANGLE; ++i)
    {
        valeurs[i] = ValeurPolaire(std::cos(i * MUL_TO_RAD), std::sin(i * MUL_TO_RAD));
    }
}

/******************************************************************************
* Retourne une valeur polaire de l'angle fourni.
*
* @param angle      [in] Un angle compris entre [0, 180[ (180 = 2*PI)
* @return la valeur polaire de l'angle
******************************************************************************/
const ValeurPolaire& PolarCoordTable::operator[](unsigned char angle)
{
    return valeurs[angle];
}

/******************************************************************************
* Retourne l'angle depuis son sin et son cos
*
* @param cos    [in] le cosinus de l'angle (en radians)
* @param sin    [in] le sinus de l'angle (en radians)
* @return l'angle qui correspond au sinus et cosinus, entre 0 et 180 (pour 
*                   OpenCV)
******************************************************************************/
unsigned char PolarCoordTable::getAngle(const double &cos, const double &sin)
{
    double atan = std::atan2(sin, cos);
    if (atan < 0)
    {
        atan += 2 * M_PI;
    }
    unsigned char angle = (unsigned char)(atan * MUL_FROM_RAD);
    return angle;
}

PolarCoordTable PolarCoordTable::table = PolarCoordTable();




/******************************************************************************
* Méthode utilitaire qui devrait probablement partir d'ici. Converti une 
* couleur sous format Vec3b vers un espace de couleurs (BRG, HSV, etc...).
*
* @param pixel      [in] le pixel à convertir
* @param conversion [in] la conversion à effectuer (CV_BRG2HSV par exemple)
* @return le pixel converti
******************************************************************************/
cv::Vec3b convertVec3b(const cv::Vec3b &pixel, int conversion)
{
    static cv::Mat inputMat(1, 1, CV_8UC3);
    static cv::Mat outputMat(1, 1, CV_8UC3);

    inputMat.at<cv::Vec3b>(0) = pixel;

    cv::cvtColor(inputMat, outputMat, conversion);

    return outputMat.at<cv::Vec3b>(0);
}