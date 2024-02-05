#include "config.hpp"
#include <stdint.h>

double mapping[6][3];

void setMapping(User user, Axis axes[6][3]){

    switch(user) {
        default:
        case Danny:
	{
            double mapping_Danny[6][3] = Danny_MAP;
	    copyMapping(mapping_Danny);
            break;
	}
	case Dillon:
	{
            double mapping_Dillon[6][3] = Dillon_MAP;
	    copyMapping(mapping_Dillon);
            break;
	}
        case Zack:
	{
            double mapping_Zack[6][3] = Zack_MAP;
	    copyMapping(mapping_Zack);
            break;
	}
    }

    double scale_fact[6][3] = SCALE_FACT;

    for (uint8_t i = 0; i < 6; i++) {
	for (uint8_t j = 0; j < 3; j++) {
	    axes[i][j].set_mapping(mapping[i][j], scale_fact[i][j]);
	}
    }
}

void copyMapping(const double source[6][3]) {
    for (uint8_t i = 0; i < 6; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            mapping[i][j] = source[i][j];
        }
    }
}