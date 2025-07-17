#include "axis.hpp"
#include <stdbool.h>
#include "user_config.hpp"

#ifndef HEXA_CONFIG
#define HEXA_CONFIG

	#define STEP_THRESHOLD 40 
	#define FIFO_IDLE_THRESHOLD 100

	#ifdef DANNY	
		#define PWM_PINS {{2, 3, 4}, {5, 6, 7}, {8, 9, 10},  {11, 12, 13}, {14, 15, 18}, {19, 22, 23}}
		#define ZERO_POINTS {{2.238, 2.168, 2.188}, {2.248, 2.298, 2.318}, {2.188, 2.188, 2.188}, {2.238, 2.298, 2.208}, {2.197, 2.258, 2.218}, {2.288, 2.208, 2.238}}
		#define MAX_POS {{PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}}
		#define MIN_POS {{-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}}
		#define SCALE_FACT {{0.638, 0.638, 0.638}, {0.638, 0.638, 0.638}, {0.638, 0.638, 0.638}, {0.638, 0.638, 0.638}, {0.638, 0.638, 0.638}, {0.638, 0.638, 0.638}}
		#define REVERSE_AXIS {{true, true, true}, {true, true, true}, {true, true, true}, {true, true, true}, {true, true, true}, {true, true, true}}

	#endif

	#ifdef ZACK
	
		#define PWM_PINS {{2, 3, 4}, {5, 6, 7}, {8, 9, 10},  {11, 12, 13}, {14, 15, 18}, {19, 22, 23}}
		#define ZERO_POINTS {{2.24, 1.95, .2}, {2.3, 2.1, 1.98}, {2.68, 2.05, 2.05}, {2.7, 2.05, 2.05}, {2.45, 2.9, 1.95}, {1.57, 2.55, 2.0}} //L0S2, L0S3, L5S0 broken
		#define MAX_POS {{PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}, {PI, PI, PI}}
		#define MIN_POS {{-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}, {-PI, -PI, -PI}}
		#define SCALE_FACT {{0.75, 0.75, 1.063}, {0.75, 0.638, 0.75}, {0.75, 0.638, .638}, {0.75, 0.638, 0.638}, {0.75, 0.69, .638}, {0.75, 0.75, 0.638}}
		#define REVERSE_AXIS {{false, false, true}, {false, true, false}, {false, true, true}, {false, true, true}, {false, false, true}, {false, false, true}}
		
	#endif

	#ifdef DILLON
		#define PWM_PINS {{2, 3, 4}, {5, 6, 7}, {8, 9, 10},  {11, 12, 13}, {14, 15, 18}, {19, 22, 23}}
		//leg 6 (spare) {2.55, 2.65, 2.65}
		#define ZERO_POINTS {{2.4, 3.246, 1.9}, {2.45, 3.1, 2.3}, {2.4, 3.2, 1.35}, {2.7, 3.1, 2.35}, {2.45, 3.15, 1.6}, {2.4, 3.1, 2.3}}
		#define MAX_POS {{1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}, {1.2, PI, PI}}
		#define MIN_POS {{-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}, {-1.2, -PI, -PI}}
		#define SCALE_FACT {{0.75, 0.638, 0.75}, {0.75, 0.638, 0.75}, {0.75, 0.638, 0.75}, {0.75, 0.638, 0.75}, {0.75, 0.638, 0.75}, {0.75, 0.638, 0.75}}
		#define REVERSE_AXIS {{false, true, false}, {false, true, false}, {false, true, false}, {false, true, false}, {false, true, false}, {false, true, false}}
	#endif

#endif
