/*******************************************************************************
* Classe qui sert a interface entre l'interface graphique (d'un noeud), et la 
* configuration des fichiers de calibrations de ce noeud.
*******************************************************************************/
#ifndef RC_CALIBRATION_FILE_MANAGER_H
#define RC_CALIBRATION_FILE_MANAGER_H

#include <string>
#include <vector>

#include <ros/ros.h>


#include <elikos_remote_calib_client/Calibrator.h>

#include <elikos_msgs/GetConfigFiles.h>
#include <elikos_msgs/FileManipulation.h>

class CalibrationFileManager{
public:
    // Constructeur de la classe
    CalibrationFileManager(const std::string& nodeName);


    // Enregistre la calibration sous un nom donné
    bool saveCalibration(const std::string& fileName);
    // Charge le calibration d'un nom donné
    bool loadCalibration(const std::string& fileName);
    // Retroune le nom des fichiers de calibration
    void getCalibrationFileNames(std::vector<std::string>& fileNames);
    // Efface un fichier de calibration
    bool deleteCalibrationFile(const std::string& fileName);
    // Retourne le nom du noeud calibré
    std::string getNodeName();
private:
    std::string nodeName_;

    ros::NodeHandle nodeHandle_;
    ros::ServiceClient getConfigFilesClient_;
    ros::ServiceClient saveConfigFile_;
    ros::ServiceClient loadConfigFile_;
    ros::ServiceClient deleteConfigFile_;
};


#endif