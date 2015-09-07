#ifndef OFFSETTABLE_H_
#define OFFSETTABLE_H_

const Offset BoxOffsetTable[67] = {
		// Small boxes
		{ 0.00, 0.00 }, { 16.66, 0.0 }, { 33.32, 0.0 }, { 49.98, 0.0 }, { 66.64, 0.0 }, { 83.30, 0.0 },
		{ 99.96, 0.0 }, { 116.62, 0.0 }, { 133.28, 0.0 }, { 149.94, 0.0 }, { 166.60, 0.0 },

		{ 166.6, -14.0 }, { 149.94, -14.0 },{ 133.28, -14.0 },{ 116.62, -14.0 },{ 99.96, -14.0 },{ 83.30, -14.0 },
		{ 66.64, -14.0 }, { 49.98, -14.0 },{ 33.32, -14.0 },{ 16.66, -14.0 }, { 0.00, -14.00 },

		{ 0.00, -28.00 }, { 16.66, -28.0 }, { 33.32, -28.0 }, { 49.98, -28.0 }, { 66.64, -28.0 },
		{ 66.64, -42.0 },{ 49.98, -42.0 }, { 33.32, -42.0 }, { 16.66, -42.0 }, { 0.00, -42.00 },
		{ 0.00, -56.00 }, { 16.66, -56.0 },{ 33.32, -56.0 }, { 49.98, -56.0 }, { 66.64, -56.0 },
		{ 66.64, -70.0 },{ 49.98, -70.0 }, { 33.32, -70.0 }, { 16.66, -70.0 }, { 0.00, -70.00 },
		{ 0.00, -84.00 }, { 16.66, -84.0 }, {33.32, -84.0 },{ 49.98, -84.0 },{ 66.64, -84.0 },
		// Middle boxes
		{ 187.65, -2.0 }, { 187.65, -18.6 }, { 187.65, -35.2 }, { 187.65, -51.8 }, { 187.65, -68.4 }, { 187.65,-85 },
		{ 204.25, -2.0 }, { 204.25, -18.6 }, { 204.25,-35.2 }, { 204.25, -51.8 }, { 204.25, -68.4 }, { 204.25, -85 },
		// Large boxes
		{ 269.69, -5.0 }, { 269.69, -38.28 }, { 269.69, -71.56 }, { 269.69, -104.84 },
		{ 302.97, -5.0 }, { 302.97, -38.28 }, { 302.97, -71.56 }, { 302.97, -104.84 }};

// TODO: Y-Values have to be checked again!
// X-Offsets are set to max. x coordinates we can set!
const Offset TapeOffsetTable[20] = { 	{ 341.43, -40.0 }, { 341.43, -51.0 }, { 341.43, -62.0 }, { 341.43, -73.0 },
										{ 341.43, -84.0 }, { 341.43, -95.0 }, { 341.43, -106.0 }, { 341.43, -117.0 },
										{ 341.43,-128.0 }, { 341.43, -139.0 }, { 341.43, -150.0 }, { 341.43,-161.0 },
										{ 341.43, -172.0 }, { 341.43, -183.0 }, { 341.43,-194.0 }, { 341.43, -205.0 },
										{ 341.43, -216.0 }, { 341.43,-227.0 }, { 341.43, -238.0 }, { 341.43, -249.0 } };



#endif // OFFSETTABLE_H_
