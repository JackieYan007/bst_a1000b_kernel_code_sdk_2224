/*******************************************************************************
 * Copyright(c) Black Sesame Technologies Inc., All right reserved.
 *
 * No part of this file may be distributed, duplicated or transmitted in any
 *form or by any means, without prior consent of Black Sesame Technologies Inc.
 *
 * This file is  property. It contains BST's trade secret, proprietary
 * and confidential information.
 *
 *********************************************************************************/

#ifndef __BST_GMWARP_SCALER__
#define __BST_GMWARP_SCALER__

int8_t blackman_polyphase[] = {
	1,   127, 0,   0,   -5,  1, // M = 16
	0,   0,   0,   -1,  6,   127, 14,  124, -9, 1,   -12, 2,   0,  -2,  0,
	-4,  22,  120, 32,  114, -14, 2,   -14, 2,  0,   -4,  1,   -7, 42,  104,
	53,  95,  -14, 2,   -14, 2,   1,   -9,  1,  -11, 64,  86,  75, 75,  -12,
	1,   -11, 1,   1,   -12, 2,   -14, 86,  64, 95,  53,  -9,  1,  -7,  1,
	2,   -14, 2,   -14, 104, 42,  114, 32,  -6, 0,   -4,  0,   2,  -14, 2,
	-12, 120, 22,  124, 14,  -2,  0,   -1,  0,  1,   -9,  1,   -5, 127, 6,
	10,  114, 10,  -3,  5,   -2, // M = 18
	0,   -3,  0,   -4,  16,  113, 23,  111, 0,  -1,  -4,  -1,  0,  -5,  1,
	-7,  31,  108, 39,  103, -7,  0,   -9,  0,  1,   -8,  1,   -9, 48,  97,
	56,  90,  -10, 1,   -11, 1,   1,   -10, 1,  -11, 66,  82,  74, 74,  -11,
	1,   -11, 1,   1,   -11, 1,   -11, 82,  66, 90,  56,  -10, 1,  -9,  1,
	1,   -10, 0,   -9,  97,  48,  103, 39,  -8, 1,   -7,  1,   0,  -7,  -1,
	-4,  108, 31,  111, 23,  -5,  0,   -4,  0,  -1,  0,   -2,  5,  113, 16,
	17,  104, 17,  -5,  12,  -4, // M = 20
	0,   -5,  0,   -5,  23,  102, 29,  101, 7,  -3,  3,   -2,  0,  -6,  0,
	-7,  36,  98,  43,  95,  0,   -2,  -3,  -1, 0,   -8,  0,   -8, 50,  90,
	58,  84,  -5,  -1,  -6,  0,   0,   -8,  0,  -8,  64,  78,  71, 71,  -7,
	0,   -8,  0,   0,   -7,  0,   -6,  78,  64, 84,  58,  -8,  0,  -8,  0,
	-1,  -5,  -1,  -3,  90,  50,  95,  43,  -8, 0,   -7,  0,   -2, 0,   -2,
	3,   98,  36,  101, 29,  -6,  0,   -5,  0,  -3,  7,   -4,  12, 102, 23,
	22,  94,  22,  -5,  17,  -4, // M = 22
	0,   -5,  0,   -5,  27,  93,  33,  92,  13, -4,  9,   -3,  0,  -6,  0,
	-6,  39,  89,  45,  87,  5,   -3,  2,   -2, 0,   -6,  0,   -6, 51,  83,
	57,  78,  0,   -2,  -2,  -1,  0,   -5,  -1, -5,  63,  74,  69, 69,  -4,
	-1,  -5,  -1,  -1,  -4,  -1,  -2,  74,  63, 78,  57,  -5,  0,  -6,  0,
	-2,  0,   -2,  2,   83,  51,  87,  45,  -6, 0,   -6,  0,   -3, 5,   -3,
	9,   89,  39,  92,  33,  -6,  0,   -5,  0,  -4,  13,  -4,  17, 93,  27,
	25,  86,  25,  -4,  21,  -4, // M = 24
	0,   -4,  0,   -4,  30,  85,  35,  84,  17, -4,  13,  -3,  0,  -4,  0,
	-4,  40,  82,  45,  80,  10,  -3,  7,   -2, 0,   -4,  -1,  -3, 50,  77,
	56,  74,  4,   -2,  2,   -2,  -1,  -3,  -1, -1,  60,  70,  65, 65,  0,
	-1,  -1,  -1,  -1,  0,   -2,  2,   70,  60, 74,  56,  -3,  -1, -3,  -1,
	-2,  4,   -2,  7,   77,  50,  80,  45,  -4, 0,   -4,  0,   -3, 10,  -3,
	13,  82,  40,  84,  35,  -4,  0,   -4,  0,  -4,  17,  -4,  21, 85,  30,
	27,  80,  27,  -3,  23,  -3, // M = 26
	0,   -3,  0,   -3,  32,  79,  36,  78,  20, -3,  16,  -3,  0,  -3,  0,
	-3,  41,  77,  45,  76,  13,  -3,  10,  -2, -1,  -2,  -1,  -1, 50,  72,
	54,  70,  7,   -2,  5,   -2,  -1,  0,   -1, 1,   59,  66,  62, 62,  3,
	-1,  1,   -1,  -1,  3,   -2,  5,   66,  59, 70,  54,  0,   -1, -1,  -1,
	-2,  7,   -2,  10,  72,  50,  76,  45,  -2, -1,  -3,  0,   -3, 13,  -3,
	16,  77,  41,  78,  36,  -3,  0,   -3,  0,  -3,  20,  -3,  23, 79,  32,
	29,  74,  29,  -2,  25,  -2, // M = 28
	0,   -2,  0,   -2,  33,  74,  36,  73,  22, -2,  18,  -2,  0,  -1,  -1,
	-1,  41,  73,  45,  71,  15,  -2,  12,  -2, -1,  0,   -1,  1,  49,  69,
	53,  66,  10,  -2,  8,   -2,  -1,  2,   -1, 4,   56,  63,  60, 60,  5,
	-1,  4,   -1,  -1,  5,   -2,  8,   63,  56, 66,  53,  2,   -1, 1,   -1,
	-2,  10,  -2,  12,  69,  49,  71,  45,  0,  -1,  -1,  -1,  -2, 15,  -2,
	18,  73,  41,  73,  36,  -1,  0,   -2,  0,  -2,  22,  -2,  25, 74,  33,
	30,  70,  30,  -1,  27,  -1, // M = 30
	0,   -1,  0,   -1,  33,  70,  37,  70,  23, -2,  20,  -2,  0,  0,   -1,
	1,   41,  69,  45,  67,  17,  -2,  14,  -2, -1,  2,   -1,  3,  48,  66,
	52,  63,  12,  -2,  10,  -1,  -1,  4,   -1, 6,   54,  60,  57, 57,  8,
	-1,  6,   -1,  -1,  8,   -1,  10,  60,  54, 63,  52,  4,   -1, 3,   -1,
	-2,  12,  -2,  14,  66,  48,  67,  45,  2,  -1,  1,   -1,  -2, 17,  -2,
	20,  69,  41,  70,  37,  0,   0,   -1,  0,  -2,  23,  -1,  27, 70,  33,
	30,  68,  30,  0,   27,  0, // M = 32
	0,   0,   0,   1,   34,  66,  37,  67,  24, -1,  21,  -1,  0,  1,   -1,
	2,   41,  66,  44,  64,  19,  -1,  16,  -1, -1,  3,   -1,  4,  47,  63,
	50,  60,  14,  -1,  11,  -1,  -1,  6,   -1, 7,   53,  59,  56, 56,  9,
	-1,  7,   -1,  -1,  9,   -1,  11,  59,  53, 60,  50,  6,   -1, 4,   -1,
	-1,  14,  -1,  16,  63,  47,  64,  44,  3,  -1,  2,   -1,  -1, 19,  -1,
	21,  66,  41,  67,  37,  1,   0,   1,   0,  -1,  24,  0,   27, 66,  34,
	31,  64,  31,  1,   28,  0, // M = 34
	0,   1,   0,   2,   34,  64,  37,  64,  25, 0,   23,  0,   0,  2,   -1,
	3,   40,  63,  44,  62,  20,  -1,  17,  -1, -1,  4,   -1,  6,  46,  61,
	49,  59,  15,  -1,  13,  -1,  -1,  7,   -1, 9,   52,  56,  54, 54,  11,
	-1,  9,   -1,  -1,  11,  -1,  13,  56,  52, 59,  49,  7,   -1, 6,   -1,
	-1,  15,  -1,  17,  61,  46,  62,  44,  4,  -1,  3,   -1,  -1, 20,  0,
	23,  63,  40,  64,  37,  2,   0,   2,   0,  0,   25,  0,   28, 64,  34,
	31,  62,  31,  2,   28,  1, // M = 36
	0,   2,   0,   3,   34,  62,  37,  61,  26, 1,   23,  0,   0,  3,   0,
	4,   40,  61,  42,  59,  21,  0,   18,  0,  0,   6,   -1,  7,  45,  59,
	48,  57,  16,  0,   14,  0,   -1,  8,   -1, 10,  50,  55,  53, 53,  12,
	-1,  10,  -1,  -1,  12,  0,   14,  55,  50, 57,  48,  8,   -1, 7,   -1,
	0,   16,  0,   18,  59,  45,  59,  42,  6,  0,   4,   0,   0,  21,  0,
	23,  61,  40,  61,  37,  3,   0,   3,   0,  1,   26,  1,   28, 62,  34,
	31,  60,  31,  3,   29,  2, // M = 38
	0,   3,   0,   3,   34,  60,  37,  60,  26, 1,   24,  1,   0,  4,   0,
	5,   39,  59,  42,  58,  21,  1,   19,  0,  0,   6,   0,   8,  45,  56,
	47,  55,  17,  0,   15,  0,   0,   9,   0,  11,  49,  53,  51, 51,  13,
	0,   11,  0,   0,   13,  0,   15,  53,  49, 55,  47,  9,   0,  8,   0,
	0,   17,  0,   19,  56,  45,  58,  42,  6,  0,   5,   0,   1,  21,  1,
	24,  59,  39,  60,  37,  4,   0,   3,   0,  1,   26,  2,   29, 60,  34,
	32,  58,  32,  3,   29,  3, // M = 40
	0,   3,   0,   4,   34,  58,  37,  58,  26, 2,   24,  1,   0,  5,   0,
	6,   39,  58,  42,  56,  22,  1,   19,  1,  0,   7,   0,   9,  44,  55,
	46,  54,  18,  0,   15,  0,   0,   10,  0,  12,  49,  52,  50, 50,  14,
	0,   12,  0,   0,   14,  0,   15,  52,  49, 54,  46,  10,  0,  9,   0,
	0,   18,  1,   19,  55,  44,  56,  42,  7,  0,   6,   0,   1,  22,  1,
	24,  58,  39,  58,  37,  5,   0,   4,   0,  2,   26,  3,   29, 58,  34,
	31,  58,  31,  4,   29,  3, // M = 42
	0,   4,   0,   5,   34,  57,  36,  57,  27, 2,   24,  2,   0,  6,   0,
	7,   39,  56,  42,  55,  22,  1,   20,  1,  0,   8,   0,   9,  44,  54,
	45,  53,  18,  1,   16,  0,   0,   11,  0,  13,  48,  51,  50, 50,  14,
	0,   13,  0,   0,   14,  0,   16,  51,  48, 53,  45,  11,  0,  9,   0,
	1,   18,  1,   20,  54,  44,  55,  42,  8,  0,   7,   0,   1,  22,  2,
	24,  56,  39,  57,  36,  6,   0,   5,   0,  2,   27,  3,   29, 57,  34,
	32,  56,  32,  4,   29,  4, // M = 44
	0,   4,   0,   5,   34,  56,  36,  56,  27, 3,   25,  2,   0,  6,   0,
	7,   39,  55,  40,  54,  23,  2,   21,  1,  0,   9,   0,   10, 43,  53,
	45,  52,  19,  1,   17,  1,   0,   11,  0,  13,  47,  50,  49, 49,  15,
	0,   13,  0,   0,   15,  1,   17,  50,  47, 52,  45,  11,  0,  10,  0,
	1,   19,  1,   21,  53,  43,  54,  40,  9,  0,   7,   0,   2,  23,  2,
	25,  55,  39,  56,  36,  6,   0,   5,   0,  3,   27,  4,   29, 56,  34,
	32,  54,  32,  5,   29,  4, // M = 46
	0,   5,   0,   6,   34,  55,  36,  55,  27, 3,   25,  3,   0,  7,   0,
	8,   38,  54,  41,  53,  23,  2,   21,  2,  0,   9,   0,   11, 42,  52,
	45,  51,  19,  1,   17,  1,   0,   12,  1,  14,  46,  49,  48, 48,  15,
	1,   14,  1,   1,   15,  1,   17,  49,  46, 51,  45,  12,  0,  11,  0,
	1,   19,  2,   21,  52,  42,  53,  41,  9,  0,   8,   0,   2,  23,  3,
	25,  54,  38,  55,  36,  7,   0,   6,   0,  3,   27,  4,   29, 55,  34,
	32,  54,  32,  5,   30,  4, // M = 48
	0,   5,   0,   6,   34,  54,  36,  53,  28, 4,   26,  3,   0,  7,   0,
	8,   38,  53,  40,  53,  23,  2,   21,  2,  0,   10,  0,   11, 42,  52,
	44,  50,  19,  2,   17,  1,   0,   13,  1,  14,  46,  49,  47, 47,  16,
	1,   14,  1,   1,   16,  1,   17,  49,  46, 50,  44,  13,  0,  11,  0,
	2,   19,  2,   21,  52,  42,  53,  40,  10, 0,   8,   0,   2,  23,  3,
	26,  53,  38,  53,  36,  7,   0,   6,   0,  4,   28,  4,   30, 54,  34,
	31,  54,  31,  6,   29,  5, // M = 50
	0,   6,   0,   7,   34,  53,  36,  52,  28, 4,   26,  3,   0,  8,   0,
	9,   38,  52,  40,  51,  24,  3,   22,  2,  0,   10,  0,   11, 42,  51,
	43,  49,  20,  2,   18,  1,   1,   13,  1,  15,  45,  48,  47, 47,  16,
	1,   15,  1,   1,   16,  1,   18,  48,  45, 49,  43,  13,  1,  11,  0,
	2,   20,  2,   22,  51,  42,  51,  40,  10, 0,   9,   0,   3,  24,  3,
	26,  52,  38,  52,  36,  8,   0,   7,   0,  4,   28,  5,   29, 53,  34,
	32,  52,  32,  6,   30,  5, // M = 52
	0,   6,   0,   7,   34,  52,  36,  52,  28, 4,   26,  4,   0,  8,   0,
	9,   38,  51,  40,  51,  24,  3,   22,  2,  0,   10,  1,   12, 41,  50,
	43,  49,  20,  2,   18,  2,   1,   13,  1,  15,  45,  47,  46, 46,  17,
	1,   15,  1,   1,   17,  2,   18,  47,  45, 49,  43,  13,  1,  12,  1,
	2,   20,  2,   22,  50,  41,  51,  40,  10, 0,   9,   0,   3,  24,  4,
	26,  51,  38,  52,  36,  8,   0,   7,   0,  4,   28,  5,   30, 52,  34,
	32,  52,  32,  6,   30,  5, // M = 54
	0,   6,   0,   7,   34,  52,  36,  51,  28, 5,   26,  4,   0,  8,   0,
	9,   38,  51,  39,  50,  24,  3,   22,  3,  1,   11,  1,   12, 41,  49,
	43,  48,  20,  2,   18,  2,   1,   14,  1,  15,  45,  47,  46, 46,  17,
	1,   15,  1,   1,   17,  2,   18,  47,  45, 48,  43,  14,  1,  12,  1,
	2,   20,  3,   22,  49,  41,  50,  39,  11, 1,   9,   0,   3,  24,  4,
	26,  51,  38,  51,  36,  8,   0,   7,   0,  5,   28,  5,   30, 52,  34
};

int8_t kaiser_polyphase[] = { 0,  64, 0,   0, -3,  1, 0, 0,   0, -1,  4,  63,
			      8,  61, -6,  2, -8,  2, 1, -2,  1, -4,  13, 60,
			      18, 56, -9,  3, -10, 3, 1, -5,  2, -7,  23, 53,
			      28, 49, -10, 3, -10, 3, 2, -8,  3, -9,  33, 44,
			      39, 39, -10, 3, -9,  3, 3, -10, 3, -10, 44, 33,
			      49, 28, -8,  2, -7,  2, 3, -10, 3, -10, 53, 23,
			      56, 18, -5,  1, -4,  1, 3, -9,  2, -8,  60, 13,
			      61, 8,  -2,  1, -1,  0, 2, -6,  1, -3,  63, 4 };

int8_t gausswin_polyphase[] = {
	1,   127, 0,   0,   -6, 1,   0,   0,   0,   -2,  8,   127, 15,  123,
	-10, 2,   -13, 3,   1,  -3,  1,   -5,  23,  119, 33,  112, -15, 4,
	-17, 4,   2,   -8,  2,  -10, 44,  105, 54,  96,  -17, 4,   -16, 4,
	3,   -12, 3,   -14, 65, 86,  76,  76,  -15, 3,   -14, 3,   3,   -15,
	4,   -16, 86,  65,  96, 54,  -12, 3,   -10, 2,   4,   -17, 4,   -17,
	105, 44,  112, 33,  -8, 2,   -5,  1,   4,   -15, 3,   -13, 119, 23,
	123, 15,  -3,  1,   -2, 0,   2,   -10, 1,   -6,  127, 8
};

#endif