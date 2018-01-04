/*******************************************************************************
* Implémentation de la classe CalibrationFileManager
*******************************************************************************/
#include "elikos_remote_calib/model/CalibrationFileManager.h"


/*******************************************************************************
* Constructeur de la classe CalibrationFileManager. Se connecte automatiquement 
* au noeud fourni.
*
* @param nodeName   [in] Le nom du noeud auquel se connecter
*******************************************************************************/
CalibrationFileManager::CalibrationFileManager(const std::string& nodeName)
    : nodeHandle_()
    , nodeName_(nodeName)
{
    getConfigFilesClient_ 
        = nodeHandle_.serviceClient<elikos_msgs::GetConfigFiles>(
            nodeName + '/' + 
            remote_calib::REMOTE_CALIB_NAMESPCACE + '/' + 
            remote_calib::GET_FILE_NAME_SERVICE_NAME
        );
    
    saveConfigFile_
        = nodeHandle_.serviceClient<elikos_msgs::FileManipulation>(
            nodeName + '/' + 
            remote_calib::REMOTE_CALIB_NAMESPCACE + '/' + 
            remote_calib::SAVE_SERVICE_NAME
        );

    loadConfigFile_
        = nodeHandle_.serviceClient<elikos_msgs::FileManipulation>(
            nodeName + '/' + 
            remote_calib::REMOTE_CALIB_NAMESPCACE + '/' + 
            remote_calib::LOAD_SERVICE_NAME
        );
    deleteConfigFile_
        = nodeHandle_.serviceClient<elikos_msgs::FileManipulation>(
            nodeName + '/' + 
            remote_calib::REMOTE_CALIB_NAMESPCACE + '/' + 
            remote_calib::DELETE_FILE_SERVICE_NAME
        );

}


/*******************************************************************************
* Envoie un message au noeud calibré qui lui dit d'enregistrer la configuration 
* courrante sous le nom 'fileName'.
*
* @param fileName   [in] le nom du fichier a enregistrer
* @return si l'enregistrement de fichier a échoué ou non.
*******************************************************************************/
bool CalibrationFileManager::saveCalibration(const std::string& fileName)
{
    elikos_msgs::FileManipulation saveFile;
    saveFile.request.fileName = fileName;

    if(saveConfigFile_.call(saveFile)){
        return saveFile.response.sucess;
    } else {
        return false;
    }
}

/*******************************************************************************
* Envoie un message au noeud calibré qui lui dit d'enregistrer la configuration 
* 'fileName'.
*
* @param fileName   [in] le nom du fichier a charger
* @return si le chargement du fichier a échoué ou non.
*******************************************************************************/
bool CalibrationFileManager::loadCalibration(const std::string& fileName)
{
    elikos_msgs::FileManipulation loadFile;
    loadFile.request.fileName = fileName;

    if(loadConfigFile_.call(loadFile)){
        return loadFile.response.sucess;
    } else {
        return false;
    }
}

/*******************************************************************************
* Retrouve le nom des fichiers de configuration d'un noeud.
*
* @param fileNames     [out] les noms des fichiers de configuration
*******************************************************************************/
void CalibrationFileManager::getCalibrationFileNames(std::vector<std::string>& fileNames)
{
    fileNames.clear();
    elikos_msgs::GetConfigFiles cfgFiles;
    if(getConfigFilesClient_.call(cfgFiles)){
        fileNames = cfgFiles.response.fileNames;
    }
}

/*******************************************************************************
* Essai d'effacer un ficher de calibraion.
*
* @param fileName   [in] le nom du ficher a effacer
* @return si l'effacement a réussi
*******************************************************************************/
bool CalibrationFileManager::deleteCalibrationFile(const std::string& fileName)
{
    elikos_msgs::FileManipulation deleteFile;
    deleteFile.request.fileName = fileName;

    if(deleteConfigFile_.call(deleteFile)){
        return deleteFile.response.sucess;
    } else {
        return false;
    }
}

/*******************************************************************************
* Retroune le nom du noeud calibré
* 
* @return le nom du noeud calibré
*******************************************************************************/
std::string CalibrationFileManager::getNodeName()
{
    return nodeName_;
}