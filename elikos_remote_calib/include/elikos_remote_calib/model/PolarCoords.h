/******************************************************************************
* Classe qui sert a calculer des coordonés polaires sans calculer des sins et 
* des cos. On pense qu'elle est plus rapide, mais on a jamais essayé... On 
* prie encore aujourd'hui qu'on ait pas fait plein de travail pour rien.
******************************************************************************/

#include <cmath>
#include <opencv2/opencv.hpp>

/******************************************************************************
* Une structure qui contient des valeurs polaires, c'est-à-dire le sinus et le
* cosinius d'un angle
******************************************************************************/
struct ValeurPolaire
{
    ValeurPolaire();
    ValeurPolaire(double cos, double sin);

    double cosinus;
    double sinus;
};


/******************************************************************************
* Classe qui gere les coordonés polaires et les angles. Évite de calculer des 
* sin et des cos 
******************************************************************************/
class PolarCoordTable
{
private:
    PolarCoordTable();

public:
    const ValeurPolaire& operator[](unsigned char angle);
    unsigned char getAngle(const double &cos, const double &sin);

    static PolarCoordTable table;

private:
    static const unsigned MAX_ANGLE = 180;
    const double MUL_TO_RAD = M_PI / 90.0;
    const double MUL_FROM_RAD = 1 / MUL_TO_RAD;
    ValeurPolaire valeurs[MAX_ANGLE];
};

/******************************************************************************
* Méthode utilitaire qui devrait probablement partir d'ici. Converti une 
* couleur sous format Vec3b vers un espace de couleurs (BRG, HSV, etc...).
*
* @param pixel      [in] le pixel à convertir
* @param conversion [in] la conversion à effectuer (CV_BRG2HSV par exemple)
* @return le pixel converti
******************************************************************************/
cv::Vec3b convertVec3b(const cv::Vec3b &pixel, int conversion);


