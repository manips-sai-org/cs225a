#include "simviz/SimVizRedisInterface.h"
#include "simviz/SimVizConfigParser.h"

int main(int argc, char** argv) {
	SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = std::string(CS225A_URDF_FOLDER);
    SaiModel::URDF_FOLDERS["HW2_FOLDER"] = std::string(HW_FOLDER) + "/hw2";
    std::string config_file = std::string(HW_FOLDER) + "/hw2/simviz_config.xml";
    SaiInterfaces::SimVizConfigParser parser;
    SaiInterfaces::SimVizRedisInterface simviz(parser.parseConfig(config_file));
    simviz.run();
    return 0;
}
