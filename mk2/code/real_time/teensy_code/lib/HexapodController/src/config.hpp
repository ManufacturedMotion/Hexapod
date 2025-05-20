#include "axis.hpp"
#include <stdbool.h>
#include "user_config.hpp"

#ifndef HEXA_CONFIG
#define HEXA_CONFIG

	#define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100

	#ifdef DANNY	
		#define PWM_PINS {{2, 3, 4}, {5, 6, 7}, {8, 9, 10},  {11, 12, 13}, {14, 15, 18}, {19, 22, 23}}
		#define ZERO_POINTS {{2.05, 2.3, 2.85}, {1.87, 2.30, 2.8}, {1.75, 2.2, 2.75}, {2.25, 2.15, 2.6}, {2.1, 2.4, 2.8}, {2.15, 2.35, 2.53}}
		#define MAX_POS {{PI/8.00, PI, PI}, {PI/8.00, PI, PI}, {PI/8.00, PI, PI}, {PI/8.00, PI, PI}, {PI/8.00, PI, PI}, {PI/8.00, PI, PI}}
		#define MIN_POS {{-PI/8.00, -PI, -PI}, {-PI/8.00, -PI, -PI}, {-PI/8.00, -PI, -PI}, {-PI/8.00, -PI, -PI}, {-PI/8.00, -PI, -PI}, {-PI/8.00, -PI, -PI}}
		#define SCALE_FACT {{0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}}
		#define REVERSE_AXIS {{false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}

	#endif

	#ifdef ZACK
	
		#define PWM_PINS {{2, 3, 4}, {5, 6, 7}, {8, 9, 10},  {11, 12, 13}, {14, 15, 18}, {19, 22, 23}}
		#define ZERO_POINTS {{2.24, 1.95, .57}, {2.3, 2.3, 1.98}, {2.4, 2.6, 2.38}, {2.7, 2.55, 1.1}, {2.45, 2.65, 1.5}, {1.57, 2.55, 2.15}} //L0S2, L0S3, L5S0 broken
		#define MAX_POS {{PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}}
		#define MIN_POS {{-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}}
		#define SCALE_FACT {{0.75, 0.75, 1.063}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 1.045}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}}
		#define REVERSE_AXIS {{false, false, true}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}, {false, false, false}}
		
	#endif

	#ifdef DILLON
		#define PWM_PINS {{2, 3, 4}, {5, 6, 7}, {8, 9, 10},  {11, 12, 13}, {14, 15, 18}, {19, 22, 23}}
		//#define ZERO_POINTS {{2.4, 2.65, 2.1}, {2.45, 2.65, 2.3}, {2.4, 2.6, 2.05}, {2.7, 2.6, 2.35}, {2.45, 2.7, 1.9}, {2.4, 2.65, 2.3}}
		#define ZERO_POINTS {{2.4, 2.696, 2.1}, {2.45, 2.65, 2.3}, {2.4, 2.77, 2.05}, {2.7, 2.65, 2.35}, {2.45, 2.65, 1.9}, {2.4, 2.65, 2.3}}
		#define MAX_POS {{1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}}
		#define MIN_POS {{-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}}
		#define SCALE_FACT {{0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}, {0.75, 0.75, 0.75}}
		#define REVERSE_AXIS {{false, true, false}, {false, true, false}, {false, true, false}, {false, true, false}, {false, true, false}, {false, true, false}}
	#endif

#endif
