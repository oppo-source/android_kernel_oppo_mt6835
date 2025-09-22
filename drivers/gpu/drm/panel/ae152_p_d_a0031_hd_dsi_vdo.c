/********************************************
 ** Copyright (C) 2025 OPLUS Mobile Comm Corp. Ltd.
 ** OPLUS_EDIT
 ** File: ae152_p_d_a0031_hd_dsi_vdo.c
 ** Description: Source file for LCD driver To Control LCD driver
 ** Version : 1.0
 ** Date : 2025/05/30
 ** ---------------- Revision History: --------------------------
 ** <version: >        <date>                  < author >                          <desc>
 ********************************************/

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>
#include <soc/oplus/system/boot_mode.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#define CONFIG_MTK_PANEL_EXT
#if defined(CONFIG_MTK_PANEL_EXT)
#include "../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
#include "../mediatek/mediatek_v2/mtk_mipi_tx.h"
#endif

#if IS_ENABLED(CONFIG_DRM_OPLUS_PANEL_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_PANEL_NOTIFY)
#include <linux/soc/qcom/panel_event_notifier.h>
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_DRM_MSM) || IS_ENABLED(CONFIG_DRM_OPLUS_NOTIFY)
#include <linux/msm_drm_notify.h>
#elif IS_ENABLED(CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY)
#include <linux/mtk_panel_ext.h>
#include <linux/mtk_disp_notify.h>
#endif
#ifdef CONFIG_TOUCHPANEL_MTK_PLATFORM
#ifndef CONFIG_OPLUS_MTK_DRM_GKI_NOTIFY
extern enum boot_mode_t get_boot_mode(void);
#endif
#else
extern int get_boot_mode(void);
#endif
#include "ae152_p_d_a0031_hd_dsi_vdo.h"
#include "../bias/oplus23661_aw37501_bias.h"
#include <linux/reboot.h>

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
static bool aod_state = false;
extern unsigned int esd_enable;

#define MAX_NORMAL_BRIGHTNESS   3090
#define LOW_BACKLIGHT_LEVEL     8

#define LCD_CTL_TP_LOAD_FW 0x10
#define LCD_CTL_RST_ON  0x11
#define LCD_CTL_RST_OFF 0x12
#define LCD_CTL_CS_ON   0x19
#define LCD_CTL_CS_OFF  0x1A
#define LCD_CTL_IRQ_ON  0x1B
#define LCD_CTL_IRQ_OFF 0x1C

static int shutdown_lcd_drv = 0;
extern unsigned int backlight_level_esd;
/* default to launcher mode */
static int cabc_mode_backup = 3;
static int backlight_last_level = 1;
#define MIPI_DRIVER_VOLTAGE   0x6

#if IS_ENABLED(CONFIG_TOUCHPANEL_NOTIFY)
extern int (*tp_gesture_enable_notifier)(unsigned int tp_index);
#endif

static unsigned int map_exp[] = {
	   0,    1,    2,    3,    4,    5,    6,    7,    8,    9,   10,   11,   12,   13,   14,   15,
	  16,   17,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
	  18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
	  18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
	  18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
	  18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
	  18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,   18,
	  19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
	  19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
	  19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
	  19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
	  19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,
	  19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   19,   20,   20,
	  20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
	  20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
	  20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
	  20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
	  20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,
	  20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   20,   21,   21,   21,   21,   21,
	  21,   21,   22,   22,   22,   22,   22,   22,   23,   23,   23,   23,   23,   23,   23,   24,
	  24,   24,   24,   24,   24,   25,   25,   25,   25,   25,   25,   25,   26,   26,   26,   26,
	  26,   26,   27,   27,   27,   27,   27,   27,   28,   28,   28,   28,   28,   28,   28,   29,
	  29,   29,   29,   29,   29,   30,   30,   30,   30,   30,   30,   30,   31,   31,   31,   31,
	  31,   31,   32,   32,   32,   32,   32,   32,   32,   33,   33,   33,   33,   33,   33,   34,
	  34,   34,   34,   34,   34,   35,   35,   35,   35,   35,   35,   35,   36,   36,   36,   36,
	  36,   36,   37,   37,   37,   37,   37,   37,   37,   38,   38,   38,   38,   38,   38,   39,
	  39,   39,   39,   39,   39,   39,   40,   40,   40,   40,   40,   40,   41,   41,   41,   41,
	  41,   41,   42,   42,   42,   42,   42,   43,   43,   43,   43,   44,   44,   44,   44,   45,
	  45,   45,   45,   45,   46,   46,   46,   46,   47,   47,   47,   47,   48,   48,   48,   48,
	  48,   49,   49,   49,   49,   50,   50,   50,   50,   51,   51,   51,   51,   51,   52,   52,
	  52,   52,   53,   53,   53,   53,   54,   54,   54,   54,   54,   55,   55,   55,   55,   56,
	  56,   56,   56,   57,   57,   57,   57,   57,   58,   58,   58,   58,   59,   59,   59,   59,
	  60,   60,   60,   60,   60,   61,   61,   61,   61,   62,   62,   62,   62,   63,   63,   63,
	  63,   63,   64,   64,   64,   64,   65,   65,   65,   65,   66,   66,   66,   66,   66,   67,
	  67,   67,   67,   68,   68,   68,   68,   69,   69,   69,   69,   69,   70,   70,   70,   70,
	  71,   71,   71,   71,   72,   72,   72,   72,   73,   73,   73,   74,   74,   74,   75,   75,
	  75,   76,   76,   76,   77,   77,   77,   78,   78,   78,   79,   79,   79,   80,   80,   80,
	  81,   81,   81,   81,   82,   82,   82,   83,   83,   83,   84,   84,   84,   85,   85,   85,
	  86,   86,   86,   87,   87,   87,   88,   88,   88,   89,   89,   89,   90,   90,   90,   91,
	  91,   91,   91,   92,   92,   92,   93,   93,   93,   94,   94,   94,   95,   95,   95,   96,
	  96,   96,   97,   97,   97,   98,   98,   98,   99,   99,   99,  100,  100,  100,  101,  101,
	 101,  101,  102,  102,  102,  103,  103,  103,  104,  104,  104,  105,  105,  105,  106,  106,
	 106,  107,  107,  107,  108,  108,  108,  109,  109,  109,  110,  110,  110,  111,  111,  111,
	 112,  112,  113,  113,  113,  114,  114,  115,  115,  115,  116,  116,  117,  117,  117,  118,
	 118,  119,  119,  120,  120,  120,  121,  121,  122,  122,  122,  123,  123,  124,  124,  124,
	 125,  125,  126,  126,  127,  127,  127,  128,  128,  129,  129,  129,  130,  130,  131,  131,
	 131,  132,  132,  133,  133,  134,  134,  134,  135,  135,  136,  136,  136,  137,  137,  138,
	 138,  138,  139,  139,  140,  140,  140,  141,  141,  142,  142,  143,  143,  143,  144,  144,
	 145,  145,  145,  146,  146,  147,  147,  147,  148,  148,  149,  149,  150,  150,  150,  151,
	 151,  152,  152,  152,  153,  153,  154,  154,  154,  155,  155,  156,  156,  157,  157,  157,
	 158,  158,  159,  159,  160,  160,  161,  161,  162,  162,  163,  163,  164,  164,  164,  165,
	 165,  166,  166,  167,  167,  168,  168,  169,  169,  170,  170,  171,  171,  171,  172,  172,
	 173,  173,  174,  174,  175,  175,  176,  176,  177,  177,  178,  178,  178,  179,  179,  180,
	 180,  181,  181,  182,  182,  183,  183,  184,  184,  185,  185,  186,  186,  186,  187,  187,
	 188,  188,  189,  189,  190,  190,  191,  191,  192,  192,  193,  193,  193,  194,  194,  195,
	 195,  196,  196,  197,  197,  198,  198,  199,  199,  200,  200,  200,  201,  201,  202,  202,
	 203,  203,  204,  204,  205,  205,  206,  206,  207,  207,  208,  208,  209,  209,  210,  210,
	 211,  211,  212,  212,  213,  213,  214,  214,  215,  215,  216,  217,  217,  218,  218,  219,
	 219,  220,  220,  221,  221,  222,  222,  223,  223,  224,  224,  225,  226,  226,  227,  227,
	 228,  228,  229,  229,  230,  230,  231,  231,  232,  232,  233,  233,  234,  235,  235,  236,
	 236,  237,  237,  238,  238,  239,  239,  240,  240,  241,  241,  242,  242,  243,  244,  244,
	 245,  245,  246,  246,  247,  247,  248,  248,  249,  249,  250,  250,  251,  251,  252,  252,
	 253,  254,  254,  255,  255,  256,  256,  257,  257,  258,  258,  259,  259,  260,  260,  261,
	 262,  262,  263,  263,  264,  264,  265,  266,  267,  267,  268,  268,  269,  270,  270,  271,
	 272,  272,  273,  273,  274,  275,  275,  276,  277,  277,  278,  278,  279,  280,  280,  281,
	 282,  282,  283,  284,  284,  285,  285,  286,  287,  287,  288,  289,  289,  290,  290,  291,
	 292,  292,  293,  294,  294,  295,  295,  296,  297,  297,  298,  299,  299,  300,  300,  301,
	 302,  302,  303,  304,  304,  305,  306,  306,  307,  307,  308,  309,  309,  310,  311,  311,
	 312,  312,  313,  314,  314,  315,  316,  316,  317,  317,  318,  319,  319,  320,  321,  321,
	 322,  323,  323,  324,  325,  325,  326,  327,  327,  328,  329,  329,  330,  331,  331,  332,
	 333,  333,  334,  335,  335,  336,  337,  337,  338,  339,  339,  340,  341,  341,  342,  343,
	 343,  344,  345,  345,  346,  347,  347,  348,  349,  349,  350,  351,  351,  352,  353,  353,
	 354,  355,  355,  356,  357,  357,  358,  359,  359,  360,  361,  361,  362,  363,  363,  364,
	 365,  365,  366,  367,  367,  368,  369,  369,  370,  371,  371,  372,  373,  373,  374,  375,
	 375,  376,  377,  377,  378,  379,  379,  380,  381,  381,  382,  383,  383,  384,  385,  386,
	 386,  387,  388,  388,  389,  390,  391,  391,  392,  393,  394,  394,  395,  396,  397,  397,
	 398,  399,  400,  400,  401,  402,  403,  403,  404,  405,  406,  406,  407,  408,  409,  409,
	 410,  411,  412,  412,  413,  414,  415,  415,  416,  417,  418,  418,  419,  420,  420,  421,
	 422,  423,  423,  424,  425,  426,  426,  427,  428,  429,  429,  430,  431,  432,  432,  433,
	 434,  435,  435,  436,  437,  438,  438,  439,  440,  441,  441,  442,  443,  444,  444,  445,
	 446,  447,  447,  448,  449,  450,  450,  451,  452,  453,  453,  454,  455,  456,  456,  457,
	 458,  459,  460,  460,  461,  462,  463,  463,  464,  465,  466,  467,  467,  468,  469,  470,
	 471,  471,  472,  473,  474,  474,  475,  476,  477,  478,  478,  479,  480,  481,  482,  482,
	 483,  484,  485,  485,  486,  487,  488,  489,  489,  490,  491,  492,  492,  493,  494,  495,
	 496,  496,  497,  498,  499,  500,  500,  501,  502,  503,  503,  504,  505,  506,  507,  507,
	 508,  509,  510,  511,  511,  512,  513,  514,  514,  515,  516,  517,  518,  518,  519,  520,
	 521,  522,  522,  523,  524,  525,  526,  527,  528,  528,  529,  530,  531,  532,  533,  534,
	 535,  535,  536,  537,  538,  539,  540,  541,  541,  542,  543,  544,  545,  546,  547,  548,
	 548,  549,  550,  551,  552,  553,  554,  555,  555,  556,  557,  558,  559,  560,  561,  561,
	 562,  563,  564,  565,  566,  567,  568,  568,  569,  570,  571,  572,  573,  574,  575,  575,
	 576,  577,  578,  579,  580,  581,  581,  582,  583,  584,  585,  586,  587,  588,  588,  589,
	 590,  591,  592,  593,  594,  595,  595,  596,  597,  598,  599,  600,  601,  602,  603,  604,
	 605,  605,  606,  607,  608,  609,  610,  611,  612,  613,  614,  615,  616,  616,  617,  618,
	 619,  620,  621,  622,  623,  624,  625,  626,  627,  627,  628,  629,  630,  631,  632,  633,
	 634,  635,  636,  637,  637,  638,  639,  640,  641,  642,  643,  644,  645,  646,  647,  648,
	 648,  649,  650,  651,  652,  653,  654,  655,  656,  657,  658,  659,  659,  660,  661,  662,
	 663,  664,  665,  666,  667,  668,  669,  670,  671,  672,  673,  674,  675,  676,  677,  678,
	 679,  680,  681,  682,  683,  684,  685,  686,  687,  688,  689,  690,  691,  692,  693,  694,
	 695,  696,  697,  698,  699,  700,  701,  702,  703,  704,  705,  706,  707,  708,  709,  710,
	 711,  712,  713,  714,  715,  716,  717,  718,  719,  720,  721,  722,  723,  724,  725,  726,
	 727,  728,  729,  730,  731,  732,  733,  734,  735,  736,  737,  738,  739,  740,  741,  742,
	 743,  744,  745,  746,  747,  748,  749,  750,  751,  752,  753,  754,  755,  756,  757,  758,
	 759,  760,  761,  762,  763,  764,  765,  766,  767,  768,  769,  770,  771,  772,  773,  774,
	 776,  777,  778,  779,  780,  781,  782,  783,  784,  785,  786,  787,  788,  789,  790,  791,
	 792,  793,  794,  795,  796,  797,  798,  799,  800,  801,  803,  804,  805,  806,  807,  808,
	 809,  810,  811,  812,  813,  814,  815,  816,  817,  818,  819,  820,  821,  822,  823,  824,
	 825,  826,  827,  828,  830,  831,  832,  833,  834,  835,  836,  837,  838,  840,  841,  842,
	 843,  844,  845,  846,  847,  849,  850,  851,  852,  853,  854,  855,  856,  858,  859,  860,
	 861,  862,  863,  864,  865,  867,  868,  869,  870,  871,  872,  873,  874,  876,  877,  878,
	 879,  880,  881,  882,  883,  885,  886,  887,  888,  889,  890,  891,  892,  894,  895,  896,
	 897,  898,  899,  900,  901,  903,  904,  905,  906,  907,  908,  909,  910,  912,  913,  914,
	 915,  916,  917,  919,  920,  921,  922,  923,  924,  926,  927,  928,  929,  930,  931,  933,
	 934,  935,  936,  937,  938,  940,  941,  942,  943,  944,  945,  947,  948,  949,  950,  951,
	 952,  954,  955,  956,  957,  958,  959,  961,  962,  963,  964,  965,  966,  968,  969,  970,
	 971,  972,  973,  975,  976,  977,  978,  979,  980,  982,  983,  984,  985,  986,  987,  989,
	 990,  991,  992,  993,  994,  996,  997,  998,  999, 1000, 1001, 1002, 1004, 1005, 1006, 1007,
	1008, 1009, 1011, 1012, 1013, 1014, 1015, 1016, 1018, 1019, 1020, 1021, 1022, 1023, 1025, 1026,
	1027, 1028, 1029, 1030, 1032, 1033, 1034, 1035, 1036, 1037, 1039, 1040, 1041, 1042, 1043, 1044,
	1046, 1047, 1048, 1049, 1050, 1051, 1053, 1054, 1055, 1056, 1057, 1058, 1060, 1061, 1062, 1063,
	1064, 1065, 1067, 1068, 1069, 1070, 1071, 1072, 1074, 1075, 1076, 1077, 1078, 1079, 1081, 1082,
	1083, 1084, 1086, 1087, 1088, 1089, 1091, 1092, 1093, 1094, 1096, 1097, 1098, 1099, 1101, 1102,
	1103, 1104, 1106, 1107, 1108, 1110, 1111, 1112, 1113, 1115, 1116, 1117, 1118, 1120, 1121, 1122,
	1123, 1125, 1126, 1127, 1128, 1130, 1131, 1132, 1133, 1135, 1136, 1137, 1139, 1140, 1141, 1142,
	1144, 1145, 1146, 1147, 1149, 1150, 1151, 1152, 1154, 1155, 1156, 1157, 1159, 1160, 1161, 1162,
	1164, 1165, 1166, 1168, 1169, 1170, 1172, 1173, 1174, 1176, 1177, 1178, 1180, 1181, 1182, 1184,
	1185, 1186, 1188, 1189, 1190, 1192, 1193, 1194, 1196, 1197, 1198, 1200, 1201, 1202, 1204, 1205,
	1206, 1208, 1209, 1210, 1212, 1213, 1214, 1216, 1217, 1218, 1220, 1221, 1222, 1224, 1225, 1226,
	1228, 1229, 1230, 1232, 1233, 1234, 1236, 1237, 1238, 1240, 1241, 1242, 1244, 1245, 1246, 1248,
	1249, 1250, 1252, 1253, 1254, 1256, 1257, 1258, 1260, 1261, 1262, 1264, 1265, 1266, 1268, 1269,
	1270, 1272, 1273, 1275, 1276, 1277, 1279, 1280, 1281, 1283, 1284, 1285, 1287, 1288, 1289, 1291,
	1292, 1294, 1295, 1296, 1298, 1299, 1300, 1302, 1303, 1304, 1306, 1307, 1308, 1310, 1311, 1313,
	1314, 1315, 1317, 1318, 1319, 1321, 1322, 1323, 1325, 1326, 1327, 1329, 1330, 1332, 1333, 1334,
	1336, 1337, 1338, 1340, 1341, 1342, 1344, 1345, 1347, 1348, 1350, 1351, 1353, 1354, 1356, 1358,
	1359, 1361, 1362, 1364, 1365, 1367, 1369, 1370, 1372, 1373, 1375, 1376, 1378, 1380, 1381, 1383,
	1384, 1386, 1387, 1389, 1391, 1392, 1394, 1395, 1397, 1398, 1400, 1402, 1403, 1405, 1406, 1408,
	1409, 1411, 1413, 1414, 1416, 1417, 1419, 1420, 1422, 1424, 1425, 1427, 1428, 1430, 1431, 1433,
	1435, 1436, 1438, 1439, 1441, 1442, 1444, 1446, 1447, 1448, 1450, 1451, 1453, 1454, 1455, 1457,
	1458, 1460, 1461, 1463, 1464, 1465, 1467, 1468, 1470, 1471, 1473, 1474, 1475, 1477, 1478, 1480,
	1481, 1482, 1484, 1485, 1487, 1488, 1490, 1491, 1492, 1494, 1495, 1497, 1498, 1500, 1501, 1502,
	1504, 1505, 1507, 1508, 1509, 1511, 1512, 1514, 1515, 1517, 1518, 1519, 1521, 1522, 1524, 1525,
	1527, 1528, 1529, 1531, 1532, 1534, 1535, 1537, 1538, 1540, 1541, 1543, 1545, 1546, 1548, 1549,
	1551, 1553, 1554, 1556, 1558, 1559, 1561, 1562, 1564, 1566, 1567, 1569, 1570, 1572, 1574, 1575,
	1577, 1579, 1580, 1582, 1583, 1585, 1587, 1588, 1590, 1591, 1593, 1595, 1596, 1598, 1600, 1601,
	1603, 1604, 1606, 1608, 1609, 1611, 1612, 1614, 1616, 1617, 1619, 1621, 1622, 1624, 1625, 1627,
	1629, 1630, 1632, 1634, 1635, 1637, 1638, 1640, 1641, 1643, 1644, 1646, 1647, 1649, 1650, 1652,
	1653, 1655, 1656, 1658, 1659, 1661, 1662, 1664, 1666, 1667, 1669, 1670, 1672, 1673, 1675, 1676,
	1678, 1679, 1681, 1682, 1684, 1685, 1687, 1688, 1690, 1691, 1693, 1694, 1696, 1698, 1699, 1701,
	1702, 1704, 1705, 1707, 1708, 1710, 1711, 1713, 1714, 1716, 1717, 1719, 1720, 1722, 1723, 1725,
	1727, 1728, 1730, 1731, 1733, 1735, 1736, 1738, 1739, 1741, 1743, 1744, 1746, 1748, 1749, 1751,
	1752, 1754, 1756, 1757, 1759, 1760, 1762, 1764, 1765, 1767, 1769, 1770, 1772, 1773, 1775, 1777,
	1778, 1780, 1781, 1783, 1785, 1786, 1788, 1790, 1791, 1793, 1794, 1796, 1798, 1799, 1801, 1802,
	1804, 1806, 1807, 1809, 1811, 1812, 1814, 1815, 1817, 1819, 1820, 1822, 1824, 1825, 1827, 1829,
	1830, 1832, 1834, 1835, 1837, 1839, 1841, 1842, 1844, 1846, 1847, 1849, 1851, 1852, 1854, 1856,
	1858, 1859, 1861, 1863, 1864, 1866, 1868, 1869, 1871, 1873, 1875, 1876, 1878, 1880, 1881, 1883,
	1885, 1886, 1888, 1890, 1892, 1893, 1895, 1897, 1898, 1900, 1902, 1903, 1905, 1907, 1909, 1910,
	1912, 1914, 1915, 1917, 1919, 1921, 1922, 1924, 1926, 1927, 1929, 1931, 1932, 1934, 1936, 1938,
	1939, 1941, 1943, 1944, 1946, 1948, 1950, 1951, 1953, 1955, 1956, 1958, 1960, 1961, 1963, 1965,
	1967, 1968, 1970, 1972, 1973, 1975, 1977, 1979, 1980, 1982, 1984, 1985, 1987, 1989, 1990, 1992,
	1994, 1996, 1997, 1999, 2001, 2002, 2004, 2006, 2008, 2009, 2011, 2013, 2014, 2016, 2018, 2020,
	2021, 2023, 2025, 2027, 2028, 2030, 2032, 2034, 2035, 2037, 2039, 2041, 2042, 2044, 2046, 2048,
	2049, 2051, 2053, 2055, 2056, 2058, 2060, 2062, 2063, 2065, 2067, 2069, 2070, 2072, 2074, 2076,
	2077, 2079, 2081, 2083, 2084, 2086, 2088, 2090, 2091, 2093, 2095, 2097, 2098, 2100, 2102, 2104,
	2105, 2107, 2109, 2111, 2112, 2114, 2116, 2118, 2120, 2121, 2123, 2125, 2127, 2129, 2130, 2132,
	2134, 2136, 2138, 2140, 2141, 2143, 2145, 2147, 2149, 2150, 2152, 2154, 2156, 2158, 2160, 2161,
	2163, 2165, 2167, 2169, 2170, 2172, 2174, 2176, 2178, 2180, 2181, 2183, 2185, 2187, 2189, 2190,
	2192, 2194, 2196, 2198, 2200, 2201, 2203, 2205, 2207, 2209, 2210, 2212, 2214, 2216, 2218, 2220,
	2221, 2223, 2225, 2227, 2229, 2231, 2233, 2235, 2237, 2238, 2240, 2242, 2244, 2246, 2248, 2250,
	2252, 2254, 2255, 2257, 2259, 2261, 2263, 2265, 2267, 2269, 2271, 2272, 2274, 2276, 2278, 2280,
	2282, 2284, 2286, 2288, 2289, 2291, 2293, 2295, 2297, 2299, 2301, 2303, 2305, 2306, 2308, 2310,
	2312, 2314, 2316, 2318, 2320, 2322, 2323, 2325, 2327, 2329, 2331, 2333, 2335, 2337, 2339, 2341,
	2342, 2344, 2346, 2348, 2350, 2352, 2354, 2356, 2358, 2360, 2362, 2363, 2365, 2367, 2369, 2371,
	2373, 2375, 2377, 2379, 2381, 2383, 2384, 2386, 2388, 2390, 2392, 2394, 2396, 2398, 2400, 2402,
	2404, 2405, 2407, 2409, 2411, 2413, 2415, 2417, 2419, 2421, 2423, 2425, 2426, 2428, 2430, 2432,
	2434, 2436, 2438, 2439, 2441, 2443, 2445, 2447, 2449, 2451, 2453, 2454, 2456, 2458, 2460, 2462,
	2464, 2466, 2468, 2469, 2471, 2473, 2475, 2477, 2479, 2481, 2483, 2484, 2486, 2488, 2490, 2492,
	2494, 2496, 2498, 2499, 2501, 2503, 2505, 2507, 2509, 2511, 2513, 2514, 2516, 2518, 2520, 2522,
	2524, 2526, 2528, 2530, 2532, 2534, 2536, 2538, 2540, 2542, 2544, 2546, 2548, 2550, 2552, 2554,
	2556, 2558, 2560, 2562, 2564, 2566, 2568, 2570, 2572, 2574, 2576, 2578, 2580, 2582, 2584, 2586,
	2588, 2590, 2592, 2594, 2596, 2598, 2600, 2602, 2604, 2606, 2608, 2610, 2612, 2614, 2616, 2618,
	2620, 2622, 2624, 2626, 2628, 2630, 2632, 2634, 2636, 2638, 2640, 2642, 2644, 2646, 2648, 2650,
	2652, 2654, 2656, 2658, 2660, 2662, 2664, 2666, 2668, 2670, 2672, 2674, 2676, 2678, 2680, 2682,
	2684, 2686, 2688, 2690, 2692, 2694, 2696, 2698, 2700, 2702, 2704, 2706, 2708, 2710, 2712, 2714,
	2716, 2718, 2720, 2722, 2724, 2726, 2728, 2730, 2733, 2734, 2736, 2738, 2740, 2742, 2744, 2746,
	2748, 2750, 2752, 2754, 2756, 2758, 2759, 2761, 2763, 2765, 2767, 2769, 2771, 2773, 2775, 2777,
	2779, 2781, 2783, 2784, 2786, 2788, 2790, 2792, 2794, 2796, 2798, 2800, 2802, 2804, 2806, 2808,
	2809, 2811, 2813, 2815, 2817, 2819, 2821, 2823, 2825, 2827, 2829, 2831, 2833, 2835, 2837, 2839,
	2841, 2843, 2846, 2848, 2850, 2852, 2854, 2857, 2859, 2861, 2863, 2865, 2868, 2870, 2872, 2874,
	2876, 2878, 2881, 2883, 2885, 2887, 2889, 2892, 2894, 2896, 2898, 2900, 2903, 2905, 2907, 2909,
	2911, 2913, 2916, 2918, 2920, 2922, 2924, 2927, 2929, 2931, 2933, 2935, 2938, 2940, 2942, 2944,
	2946, 2949, 2951, 2953, 2955, 2957, 2960, 2962, 2964, 2966, 2968, 2971, 2973, 2975, 2977, 2979,
	2982, 2984, 2986, 2988, 2991, 2994, 2997, 3000, 3003, 3006, 3009, 3011, 3014, 3017, 3020, 3023,
	3026, 3029, 3032, 3035, 3038, 3041, 3044, 3047, 3050, 3053, 3056, 3059, 3062, 3064, 3066, 3068,
	3070, 3072, 3074, 3075, 3076, 3077, 3078, 3079, 3080, 3081, 3082, 3083, 3084, 3085, 3086, 3087,
	3088, 3089, 3090,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0
};

#define jdi_dcs_write_seq(ctx, seq...)                                                                                 \
        ({                                                                                                                                         \
                const u8 d[] = { seq };                                                                                \
                BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 64,                                                   \
                                "DCS sequence too big for stack");                        \
                jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                                                  \
        })

#define jdi_dcs_write_seq_static(ctx, seq...)                                                                  \
        ({                                                                                                                                         \
                static const u8 d[] = { seq };                                                                 \
                jdi_dcs_write(ctx, d, ARRAY_SIZE(d));                                                  \
        })

static inline struct jdi *panel_to_jdi(struct drm_panel *panel)
{
        return container_of(panel, struct jdi, panel);
}

#ifdef PANEL_SUPPORT_READBACK
static int jdi_dcs_read(struct jdi *ctx, u8 cmd, void *data, size_t len)
{
        struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
        ssize_t ret;

        if (ctx->error < 0)
                return 0;

        ret = mipi_dsi_dcs_read(dsi, cmd, data, len);
        if (ret < 0) {
                dev_info(ctx->dev, "error %d reading dcs seq:(%#x)\n", ret,
                         cmd);
                ctx->error = ret;
        }

        return ret;
}

static void jdi_panel_get_data(struct jdi *ctx)
{
        u8 buffer[3] = { 0 };
        static int ret;

        pr_info("%s+\n", __func__);

        if (ret == 0) {
                ret = jdi_dcs_read(ctx, 0x0A, buffer, 1);
                pr_info("%s  0x%08x\n", __func__, buffer[0] | (buffer[1] << 8));
                dev_info(ctx->dev, "return %d data(0x%08x) to dsi engine\n",
                        ret, buffer[0] | (buffer[1] << 8));
        }
}
#endif

static void jdi_dcs_write(struct jdi *ctx, const void *data, size_t len)
{
        struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
        ssize_t ret;
        char *addr;

        if (ctx->error < 0)
                return;

        addr = (char *)data;

        if (len > 1)
                udelay(100);

        if ((int)*addr < 0xB0)
                ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
        else
                ret = mipi_dsi_generic_write(dsi, data, len);
        if (ret < 0) {
                dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
                ctx->error = ret;
        }
}

static void jdi_panel_init(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xB6, 0x30, 0x6b, 0x00, 0x86, 0xc3, 0x03);
	jdi_dcs_write_seq_static(ctx, 0xB7, 0x31, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xBA, 0x06, 0x90, 0x09, 0x00, 0x0b, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xBB, 0x00, 0xb4, 0xa0);
	jdi_dcs_write_seq_static(ctx, 0xBC, 0x00, 0xb4, 0xa0);
	jdi_dcs_write_seq_static(ctx, 0xBD, 0x00, 0xb4, 0xa0);
	jdi_dcs_write_seq_static(ctx, 0xBE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC0, 0x00, 0x43, 0x24, 0x06, 0x22, 0x00, 0x0a, 0x06, 0x94, 0x00, 0x70, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC1, 0x30, 0x11, 0x50, 0xfa, 0x01, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x40, 0x0f,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC2, 0x04, 0x00, 0x3c, 0x12, 0x0B, 0x10, 0x14, 0x10, 0x00, 0x04, 0x00, 0x3c, 0x12, 0x0B,
	0x10, 0x12, 0x10, 0x00, 0x04, 0x00, 0x3c, 0x12, 0x0B, 0x10, 0x16, 0x10, 0x00, 0x04, 0x00, 0x3c, 0x12, 0x0B, 0x10, 0x14,
	0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x3c, 0x10, 0x10, 0x07, 0x05, 0xc1, 0x00, 0xe0, 0x02, 0x00,
	0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0xe0, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x05, 0x00, 0x2F, 0x10, 0x10, 0x09, 0x05, 0xc1, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x11,
	0x01, 0x00, 0x02, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC3, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0x80, 0x0A, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00,
	0x3c, 0xc1, 0x01, 0xa0, 0x0A, 0x00, 0x00, 0x00, 0x20, 0x00, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0xa0, 0x0a, 0x00, 0x00, 0x00,
	0x20, 0x00, 0x04, 0x00, 0x3c, 0xc1, 0x01, 0x80, 0x0a, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC4, 0x00, 0x00, 0x00, 0x64, 0x03, 0x07, 0x21, 0x1F, 0x1D, 0x1B, 0x19, 0x17, 0x15, 0x13,
	0x5D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5D, 0x14, 0x16,
	0x18, 0x1A, 0x1C, 0x1E, 0x20, 0x22, 0x08, 0x04, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x1F, 0x00,
	0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55,
	0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55);
	jdi_dcs_write_seq_static(ctx, 0xC5, 0x08, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x01, 0x22, 0x04, 0x22, 0x01, 0x00, 0x46, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x46, 0x00, 0x01, 0x05, 0x01, 0x0b, 0x01, 0x35, 0xff, 0x0f, 0x06, 0xc5, 0x02, 0xc0, 0x0f, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x20, 0x00, 0x00, 0x00, 0xc0, 0x11, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x50, 0x00, 0x33, 0x03,
	0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xCB, 0x02, 0xd0, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff);

	jdi_dcs_write_seq_static(ctx, 0xCE, 0x16, 0x10, 0x42, 0x4b, 0x4e, 0x5c, 0x60, 0x62, 0x7f, 0x8b, 0x9f, 0xaf, 0xb6, 0xba,
	0xbf, 0xc4, 0xc9, 0x0f, 0x06, 0x04, 0x04, 0x00, 0x04, 0x8c);

	jdi_dcs_write_seq_static(ctx, 0xCF, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xD0, 0xc1, 0x50, 0x81, 0x66, 0x09, 0x90, 0x00, 0xdc, 0x92, 0x9f, 0x11, 0x3c, 0x06, 0x7e,
	0x09, 0x08, 0xd5, 0x1b, 0xf0, 0x06);
	jdi_dcs_write_seq_static(ctx, 0xD1, 0xd8, 0xd8, 0x1b, 0x33, 0x33, 0x17, 0x07, 0xbb, 0x55, 0x55, 0x55, 0x55, 0x00, 0x3b,
	0x77, 0x07, 0x3b, 0x30, 0x06, 0x72, 0x33, 0x13, 0x00, 0xd7, 0x0c, 0x55, 0x02, 0x00, 0x18, 0x70, 0x18, 0x77, 0x11, 0x11,
	0x11, 0x20, 0x20);
	jdi_dcs_write_seq_static(ctx, 0xD2, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xf7, 0xff, 0xff, 0xf7,
	0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff,
	0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7,
	0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff,
	0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff,
	0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7,
	0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff, 0xff, 0xf7, 0xff);
	jdi_dcs_write_seq_static(ctx, 0xD4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00,
	0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xD7, 0x01, 0x00, 0x12, 0x12, 0x00, 0x43, 0x00, 0x18, 0x00, 0x43, 0x00, 0x18, 0x03, 0x83,
	0x80, 0x85, 0x85, 0x85, 0x87, 0x84, 0x45, 0x86, 0x87, 0x80, 0x88, 0x86, 0x89, 0x83, 0x83, 0x87, 0x84, 0x88, 0x8A, 0x0c,
	0x0b, 0x0a, 0x0a, 0x0a, 0x07, 0x07, 0x06, 0x06, 0x00, 0x08, 0x0a, 0x0a);
	jdi_dcs_write_seq_static(ctx, 0xD8, 0x40, 0x99, 0x26, 0xed, 0x16, 0x6c, 0x16, 0x6c, 0x16, 0x6c, 0x00, 0x14, 0x00, 0x14,
	0x00, 0x14, 0x01, 0x0c, 0x00, 0x00, 0x01, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xD9, 0x01, 0x02, 0x7f, 0x22, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x0c, 0x00, 0x00, 0x0a, 0x00,
	0x00, 0x00, 0x0a, 0x00, 0x00, 0x0b);
	jdi_dcs_write_seq_static(ctx, 0xDD, 0x30, 0x06, 0x23, 0x65);
	jdi_dcs_write_seq_static(ctx, 0xDE, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xE6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xEA, 0x02, 0x07, 0x07, 0x04, 0x80, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x03, 0x23, 0x00,
	0x07, 0x00, 0xed, 0x00, 0xed, 0x00, 0xed, 0x01, 0x28, 0x01, 0x28, 0x00, 0x47, 0x00, 0x47, 0x00, 0x47, 0x01, 0x0f, 0x01,
	0x00);
	jdi_dcs_write_seq_static(ctx, 0xEB, 0x09, 0xf0, 0x9f, 0x00, 0x01, 0x01, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xEC, 0x02, 0xd0, 0x01, 0x70, 0x73, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x2a, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xED, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x26, 0xF0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x10, 0x00);
	jdi_dcs_write_seq_static(ctx, 0xEE, 0x05, 0x40, 0x01, 0x00, 0xc0, 0x0f, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x03, 0x00, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x40, 0xc0, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x05, 0xc5, 0x1f, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x00,
	0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02);
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x03);

	jdi_dcs_write_seq_static(ctx, 0x35, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x51, 0x00, 0x00);
	jdi_dcs_write_seq_static(ctx, 0x53, 0x2C);
	jdi_dcs_write_seq_static(ctx, 0x55, 0x01);
	jdi_dcs_write_seq_static(ctx, 0xB0, 0x03);

	jdi_dcs_write_seq_static(ctx, 0x11);
	usleep_range(120000, 120100);
	jdi_dcs_write_seq_static(ctx, 0x29);
	usleep_range(20000, 20100);

	pr_info("%s-\n", __func__);
}

static void cabc_mode_retore(struct jdi *ctx)
{
	jdi_dcs_write_seq_static(ctx, 0x53, 0x2c);
	if (cabc_mode_backup == 1) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x01);
	} else if (cabc_mode_backup == 2) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x02);
	} else if (cabc_mode_backup == 3) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x03);
	} else if (cabc_mode_backup == 0) {
		jdi_dcs_write_seq_static(ctx, 0x55, 0x00);
	}
	pr_info("%s- cabc_mode_backup=%d\n", __func__, cabc_mode_backup);
}

static int jdi_disable(struct drm_panel *panel)
{
        struct jdi *ctx = panel_to_jdi(panel);

        if (!ctx->enabled)
                return 0;

        if (ctx->backlight) {
                ctx->backlight->props.power = FB_BLANK_POWERDOWN;
                backlight_update_status(ctx->backlight);
        }

        ctx->enabled = false;

        return 0;
}

static int jdi_unprepare(struct drm_panel *panel)
{
	struct jdi *ctx = panel_to_jdi(panel);
	int flag_poweroff = 1;
	int blank = 0;

	if (!ctx->prepared) {
		return 0;
	}

	if (tp_gesture_enable_notifier && tp_gesture_enable_notifier(0)) {
                if (shutdown_lcd_drv == 1) {
                        flag_poweroff = 1;
                } else {
                        flag_poweroff = 0;
                        pr_info("[TP] tp gesture  is enable,Display not to poweroff\n");
                }
	} else {
		flag_poweroff = 1;
	}

	pr_info("%s+ enter jdi_unprepare \n", __func__);
        usleep_range(20000, 20100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	usleep_range(60000, 60100);
	jdi_dcs_write_seq_static(ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	usleep_range(150000, 150100);
	if (flag_poweroff == 1) {
		ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->reset_gpio, 0);
		devm_gpiod_put(ctx->dev, ctx->reset_gpio);

		usleep_range(2000, 2100);
		if (shutdown_lcd_drv == 1) {
			ctx->reset_gpio = devm_gpiod_get(ctx->dev, "tpreset", GPIOD_OUT_HIGH);
			gpiod_set_value(ctx->tpreset_gpio, 0);
			devm_gpiod_put(ctx->dev, ctx->tpreset_gpio);
			pr_info("[TP] g_shutdown_flag ==1, Display goto power off , And TP reset will low\n");
		} else {
			blank = LCD_CTL_RST_OFF;
			mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
			pr_info("[TP] tp gesture is disable, Display goto power off , And TP reset will low\n");
		}

		usleep_range(2000, 2100);
		ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_neg, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_neg);

		usleep_range(2000, 2100);
		ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
		gpiod_set_value(ctx->bias_pos, 0);
		devm_gpiod_put(ctx->dev, ctx->bias_pos);
	}
	ctx->error = 0;
	ctx->prepared = false;

	return 0;
}

static int jdi_prepare(struct drm_panel *panel)
{
        struct jdi *ctx = panel_to_jdi(panel);
        int ret;
        int mode;
        int blank = 0;

        if (ctx->prepared)
                return 0;
        pr_info("%s", __func__);

        blank = LCD_CTL_IRQ_OFF;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_info("[TP]TP LCD_CTL_IRQ_OFF will chang to off\n");
        usleep_range(2000, 2100);
        blank = LCD_CTL_RST_ON;
        mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
        pr_info("[TP]TP reset will set high\n");
        usleep_range(2000, 2100);

        ctx->bias_pos = devm_gpiod_get_index(ctx->dev, "bias", 0, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_pos, 1);
        devm_gpiod_put(ctx->dev, ctx->bias_pos);

        usleep_range(5000, 5100);
        ctx->bias_neg = devm_gpiod_get_index(ctx->dev, "bias", 1, GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->bias_neg, 1);
        devm_gpiod_put(ctx->dev, ctx->bias_neg);
        usleep_range(5000, 5100);

        lcm_i2c_write_bytes(0x0, 0x14);
        lcm_i2c_write_bytes(0x1, 0x14);

        usleep_range(3000, 3100);
        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, 1);
        usleep_range(3000, 3100);
        gpiod_set_value(ctx->reset_gpio, 0);
        usleep_range(3000, 3100);
        gpiod_set_value(ctx->reset_gpio, 1);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);
        usleep_range(22000, 22100);



        mode = get_boot_mode();
        pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
        if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
                blank = LCD_CTL_IRQ_ON;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP]TP LCD_CTL_IRQ_ON will chang to spi mode and high\n");
                usleep_range(5000, 5100);
                blank = LCD_CTL_TP_LOAD_FW;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP] start to load fw!\n");
        }

        jdi_panel_init(ctx);
        /* restore cabc mode to before suspend */
        cabc_mode_retore(ctx);

        ret = ctx->error;
        if (ret < 0)
                jdi_unprepare(panel);

        ctx->prepared = true;

        pr_info("%s-\n", __func__);
        return ret;
}


static int jdi_enable(struct drm_panel *panel)
{
        struct jdi *ctx = panel_to_jdi(panel);

        if (ctx->enabled)
                return 0;

        if (ctx->backlight) {
                ctx->backlight->props.power = FB_BLANK_UNBLANK;
                backlight_update_status(ctx->backlight);
        }

        ctx->enabled = true;

        return 0;
}

#define HFP (96)
#define HSA (4)
#define HBP (36)
#define VSA (4)
#define VBP (32)
#define VAC (1570)
#define HAC (720)
#define VFP_60hz (1972)
#define VFP_90hz (780)
#define VFP_120hz (180)
#define DYN_PLL_CLK (562)
#define DYN_DATA_RATE (1124)
#define HFP_DYN (108)
#define PLL_CLOCK (553)
#define DATA_RATE (1107)
#define PHYSICAL_WIDTH  (71474)
#define PHYSICAL_HEIGHT (155853)

static const struct drm_display_mode performance_mode_60hz = {
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_60hz + VSA + VBP) * 60) / 1000,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_60hz,
        .vsync_end = VAC + VFP_60hz + VSA,
        .vtotal = VAC + VFP_60hz + VSA + VBP,
};
static const struct drm_display_mode performance_mode_90hz = {
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_90hz + VSA + VBP) * 90) / 1000,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_90hz,
        .vsync_end = VAC + VFP_90hz + VSA,
        .vtotal = VAC + VFP_90hz + VSA + VBP,
};

static const struct drm_display_mode performance_mode_120hz = {
        .clock = ((HAC + HFP + HSA + HBP) * (VAC + VFP_120hz + VSA + VBP) * 120) / 1000,
        .hdisplay = HAC,
        .hsync_start = HAC + HFP,
        .hsync_end = HAC + HFP + HSA,
        .htotal = HAC + HFP + HSA + HBP,
        .vdisplay = VAC,
        .vsync_start = VAC + VFP_120hz,
        .vsync_end = VAC + VFP_120hz + VSA,
        .vtotal = VAC + VFP_120hz + VSA + VBP,
};

#if defined(CONFIG_MTK_PANEL_EXT)

static struct mtk_panel_params ext_params_60hz = {
                .vendor = "A0031",
                .manufacture = "P_D",
                .pll_clk = PLL_CLOCK,
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
                .lcm_esd_check_table[0] = {
                                .cmd = 0x0A,
                                .count = 1,
                                .para_list[0] = 0x9C,
                },
                .ssc_enable = 0,
                .bdg_ssc_enable = 0,
                .lane_swap_en = 0,
                .bdg_lane_swap_en = 0,
                .physical_width_um = PHYSICAL_WIDTH,
                .physical_height_um = PHYSICAL_HEIGHT,
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },
                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_60hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 12,
                },
                .oplus_display_global_dre = 1,
                .vdo_per_frame_lp_enable = 1,
                .oplus_display_lcd_tp_aod = 1,
                .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
                .cabc_three_to_zero = 1,
};

static struct mtk_panel_params ext_params_90hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "A0031",
                .manufacture = "P_D",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
                .lcm_esd_check_table[0] = {
                                .cmd = 0x0A,
                                .count = 1,
                                .para_list[0] = 0x9C,
                },
                .ssc_enable = 0,
                .bdg_ssc_enable = 0,
                .lane_swap_en = 0,
                .bdg_lane_swap_en = 0,
                .physical_width_um = PHYSICAL_WIDTH,
                .physical_height_um = PHYSICAL_HEIGHT,
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },
                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_90hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 12,
                },
                .oplus_display_global_dre = 1,
                .vdo_per_frame_lp_enable = 1,
                .oplus_display_lcd_tp_aod = 1,
                .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
                .cabc_three_to_zero = 1,
};

static struct mtk_panel_params ext_params_120hz = {
                .pll_clk = PLL_CLOCK,
                .vendor = "A0031",
                .manufacture = "P_D",
                .cust_esd_check = 1,
                .esd_check_enable = 1,
                .lcm_degree = PROBE_FROM_DTS,
                .lcm_esd_check_table[0] = {
                                .cmd = 0x0A,
                                .count = 1,
                                .para_list[0] = 0x9C,
                },
                .ssc_enable = 0,
                .bdg_ssc_enable = 0,
                .lane_swap_en = 0,
                .bdg_lane_swap_en = 0,
                .physical_width_um = PHYSICAL_WIDTH,
                .physical_height_um = PHYSICAL_HEIGHT,
                .data_rate = DATA_RATE,
                .lfr_enable = 0,
                .lfr_minimum_fps = 60,
                .dyn_fps = {
                                .switch_en = 1,
                                .vact_timing_fps = 120,
                },
                .dyn = {
                                .switch_en = 1,
                                .pll_clk = DYN_PLL_CLK,
                                .data_rate = DYN_DATA_RATE,
                                .vsa = VSA,
                                .vbp = VBP,
                                .vfp = VFP_120hz,
                                .hsa = HSA,
                                .hbp = HBP,
                                .hfp = HFP_DYN,
                },
                .phy_timcon = {
                        .hs_trail = 12,
                },
                .oplus_display_global_dre = 1,
                .vdo_per_frame_lp_enable = 1,
                .oplus_display_lcd_tp_aod = 1,
                .oplus_display_color_mode_suppor = MTK_DRM_COLOR_MODE_DISPLAY_P3,
                .cabc_three_to_zero = 1,
};

static int panel_ata_check(struct drm_panel *panel)
{
        return 1;
}

static int boe_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle,
				 unsigned int level)
{
	char bl_tb0[] = {0x51, 0x0F, 0xFF};
	char bl_tb1[] = {0x53, 0x24};
	char bl_tb2[] = {0x28};
	char bl_tb3[] = {0x29};
	int bl_map;

	if (!cb)
		return -1;

	if ((backlight_last_level == 0) && (level > 0)) {
		udelay(6000);
		cb(dsi, handle, bl_tb3, ARRAY_SIZE(bl_tb3));
		esd_enable = 1;
		pr_info("%s,backlight on", __func__);
	}

	if (level > 3710) {
		level = 3710;
	}

	oplus_display_brightness = level;
	bl_map = level;
	backlight_level_esd = level;
	backlight_last_level = level;

	if ((level > 0) && (level < MAX_NORMAL_BRIGHTNESS)) {
		bl_map = map_exp[level];
	}

	pr_info("%s, enter backlight level = %d, bl_map = %d, oplus_display_brightness = %d\n", __func__, level, bl_map, oplus_display_brightness);

	if ((aod_state == 1) && (level == 1000 || level == 1003)) {
		aod_state = false;
		return 0;
	}

	bl_tb0[1] = bl_map >> 8;
	bl_tb0[2] = bl_map & 0xFF;

	if (bl_map < LOW_BACKLIGHT_LEVEL) {
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}

	if (bl_map == 0) {
		esd_enable = 0;
		cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	pr_info("%s, exit backlight level = %d, bl_map = %d, oplus_display_brightness = %d, backlight_last_level=%d\n",
		__func__, level, bl_map, oplus_display_brightness, backlight_last_level);

	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
        struct jdi *ctx = panel_to_jdi(panel);
        int mode;
        int blank;
        char bl_tb0[] = {0x53, 0x2c};
        char bl_tb2[] = {0x55, 0x03};
        aod_state = false;

        pr_err("debug for lcm %s\n", __func__);

        if (!cb)
                return -1;

        cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
        cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

        if (!ctx->prepared) {
                return 0;
        }

        pr_err(" %s : AOD reset start\n", __func__);
        mode = get_boot_mode();
        pr_info("[TP] in dis_panel_power_on,mode = %d\n", mode);
        if ((mode != MSM_BOOT_MODE__FACTORY) &&(mode != MSM_BOOT_MODE__RF) && (mode != MSM_BOOT_MODE__WLAN)) {
                #define LCD_CTL_TP_LOAD_FW 0x10
                #define LCD_CTL_CS_ON  0x19
                #define LCD_CTL_AOD_OFF  0x30
                blank = LCD_CTL_CS_ON;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP]TP CS will chang to spi mode and high\n");
                usleep_range(5000, 5100);
              /* blank = LCD_CTL_TP_LOAD_FW;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP] start to load fw!\n");*/
                blank =  LCD_CTL_AOD_OFF;
                mtk_disp_notifier_call_chain(MTK_DISP_EVENT_FOR_TOUCH, &blank);
                pr_info("[TP] EXIT AOD!\n");
        }
        pr_err(" %s : AOD reset end\n", __func__);
        return 0;
}

static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
        char bl_tb0[] = {0x51, 0x0F, 0xFF};
        char bl_tb1[] = {0x53, 0x24};
        char bl_tb2[] = {0x55, 0x00};

        int level;
        /*50nit*/
        level = 762;
        aod_state = true;


        bl_tb0[1] = level >> 8;
        bl_tb0[2] = level & 0xFF;

        if (!cb)
                return -1;

        pr_err("debug for lcm %s\n", __func__);
        cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
        cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
        cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
        pr_info("%s, AOD backlight level = %d\n", __func__, level);
        return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
        int backlight;
        char bl_tb0[] = {0x51, 0x0F, 0xFF};
        char bl_tb2[] = {0x55, 0x00};

        pr_err("debug for lcm %s+\n", __func__);

        if (level == 0) {
                backlight = 762;

                bl_tb0[1] = backlight >> 8;
                bl_tb0[2] = backlight & 0xFF;

                if (!cb)
                        return -1;

                cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
                cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
                pr_info("%s, AOD backlight backlight = %d\n", __func__, backlight);
        } else {
                /*10nit*/
                backlight = 340;

                bl_tb0[1] = backlight >> 8;
                bl_tb0[2] = backlight & 0xFF;

                if (!cb)
                        return -1;

                cb(dsi, handle, bl_tb2, ARRAY_SIZE(bl_tb2));
                cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
                pr_info("%s, AOD backlight backlight = %d\n", __func__, backlight);
        }
        pr_err("debug for lcm %s- level = %d !\n", __func__, level);

        return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb,
		void *handle)
{
	char bl_tb0[] = {0x51, 0x03, 0xff};

	bl_tb0[1] = backlight_level_esd >> 8;
	bl_tb0[2] = backlight_level_esd & 0xFF;
	if (!cb)
		return -1;
	pr_err("%s bl_tb0[1]=%x, bl_tb0[2]=%x\n", __func__, bl_tb0[1], bl_tb0[2]);
	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));

	return 1;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
        unsigned int mode)
{
        struct drm_display_mode *m;
        unsigned int i = 0;

        list_for_each_entry(m, &connector->modes, head) {
                if (i == mode)
                        return m;
                i++;
        }
        return NULL;
}
static int mtk_panel_ext_param_set(struct drm_panel *panel,
                struct drm_connector *connector, unsigned int mode)
{
        struct mtk_panel_ext *ext = find_panel_ext(panel);
        int ret = 0;
        int target_fps;
        struct drm_display_mode *m = get_mode_by_id(connector, mode);
        target_fps = drm_mode_vrefresh(m);

        if (target_fps == 120) {
                ext->params = &ext_params_120hz;
        } else if (target_fps == 90) {
                ext->params = &ext_params_90hz;
        } else if (target_fps == 60) {
                ext->params = &ext_params_60hz;
        } else {
                pr_err("[ %s : %d ] : No mode to set fps = %d \n", __func__ ,  __LINE__ , target_fps);
                ret = 1;
        }
        return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
                struct drm_connector *connector,
                struct mtk_panel_params **ext_param, unsigned int id)
{
        int ret = 0;

        if (id == 0) {
                *ext_param = &ext_params_120hz;
        } else if (id == 1) {
                *ext_param = &ext_params_90hz;
        } else if (id == 2) {
                *ext_param = &ext_params_60hz;
        } else {
                ret = 1;
		}
        return ret;
}

static int panel_ext_reset(struct drm_panel *panel, int on)
{
        struct jdi *ctx = panel_to_jdi(panel);

        ctx->reset_gpio = devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
        gpiod_set_value(ctx->reset_gpio, on);
        devm_gpiod_put(ctx->dev, ctx->reset_gpio);

        return 0;
}

static void cabc_mode_switch(void *dsi, dcs_write_gce cb, void *handle, unsigned int cabc_mode)
{
	char bl_tb0[] = {0x53, 0x2c};
	char bl_tb1[] = {0x55, 0x00};

	pr_err("%s cabc_mode = %d\n", __func__, cabc_mode);
	if (cabc_mode > 3) {
		pr_err("%s: Invaild params skiped!\n", __func__);
		return;
	}

	cb(dsi, handle, bl_tb0, ARRAY_SIZE(bl_tb0));
	if (cabc_mode == 1) {
		bl_tb1[1] = 1;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else if (cabc_mode == 2) {
		bl_tb1[1] = 2;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else if (cabc_mode == 3) {
		bl_tb1[1] = 3;
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	} else if (cabc_mode == 0) {
		cb(dsi, handle, bl_tb1, ARRAY_SIZE(bl_tb1));
	}
	cabc_mode_backup = cabc_mode;
}
static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = boe_setbacklight_cmdq,
	.ext_param_set = mtk_panel_ext_param_set,
	.ext_param_get = mtk_panel_ext_param_get,
	.ata_check = panel_ata_check,
	.doze_enable = panel_doze_enable,
	.doze_disable = panel_doze_disable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.cabc_switch = cabc_mode_switch,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
};
#endif

struct panel_desc {
        const struct drm_display_mode *modes;
        unsigned int num_modes;

        unsigned int bpc;

        struct {
                unsigned int width;
                unsigned int height;
        } size;

        /**
         * @prepare: the time (in milliseconds) that it takes for the panel to
         *           become ready and start receiving video data
         * @enable: the time (in milliseconds) that it takes for the panel to
         *          display the first valid frame after starting to receive
         *          video data
         * @disable: the time (in milliseconds) that it takes for the panel to
         *           turn the display off (no content is visible)
         * @unprepare: the time (in milliseconds) that it takes for the panel
         *                 to power itself down completely
         */
        struct {
                unsigned int prepare;
                unsigned int enable;
                unsigned int disable;
                unsigned int unprepare;
        } delay;
};

static int jdi_get_modes(struct drm_panel *panel,
                            struct drm_connector *connector)
{
        struct drm_display_mode *mode;
        struct drm_display_mode *mode5;
        struct drm_display_mode *mode6;

        mode = drm_mode_duplicate(connector->dev, &performance_mode_60hz);
        if (!mode) {
                dev_info(connector->dev->dev, "failed to add mode1 %ux%ux@%u\n",
                         performance_mode_60hz.hdisplay, performance_mode_60hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_60hz));
                return -ENOMEM;
        }

        drm_mode_set_name(mode);
        mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode);

        mode5 = drm_mode_duplicate(connector->dev, &performance_mode_90hz);
        if (!mode5) {
                dev_info(connector->dev->dev, "failed to add mode5 %ux%ux@%u\n",
                         performance_mode_90hz.hdisplay, performance_mode_90hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_90hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode5);
        mode5->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode5);

        mode6 = drm_mode_duplicate(connector->dev, &performance_mode_120hz);
        if (!mode6) {
                dev_info(connector->dev->dev, "failed to add mode6 %ux%ux@%u\n",
                         performance_mode_120hz.hdisplay, performance_mode_120hz.vdisplay,
                         drm_mode_vrefresh(&performance_mode_120hz));
                return -ENOMEM;
        }
        drm_mode_set_name(mode6);
        mode6->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode6);

        connector->display_info.width_mm = 71;
        connector->display_info.height_mm = 155;

        return 1;
}

static const struct drm_panel_funcs jdi_drm_funcs = {
        .disable = jdi_disable,
        .unprepare = jdi_unprepare,
        .prepare = jdi_prepare,
        .enable = jdi_enable,
        .get_modes = jdi_get_modes,
};


static int lcd_vsn_reset_notify_callback(struct notifier_block *np, unsigned long type, void *_unused)
{
        switch (type) {
        case SYS_DOWN:
                shutdown_lcd_drv = 1;
                pr_info("[lcm] reboot_notify: SYS_DOWN!\n");
                break;
        case SYS_POWER_OFF:
                shutdown_lcd_drv = 1;
                pr_info("[lcm] lcd_vsn_reset_notify_callback, shutdown_lcd_drv = %d, reboot_notify: SYS_POWER_OFF!\n", shutdown_lcd_drv);
                break;

        case SYS_HALT:
                pr_info("[lcm] reboot_notify: SYS_HALT !\n");
                break;

        default:
                pr_info("[lcm] reboot_notify: default !\n");
                break;
        }
        return NOTIFY_OK;
}

static struct notifier_block lcd_vsn_reset_notifier = {
        .notifier_call = lcd_vsn_reset_notify_callback,
        .priority = 128,
};

static int jdi_probe(struct mipi_dsi_device *dsi)
{
        struct device *dev = &dsi->dev;
        struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
        struct jdi *ctx;
        struct device_node *backlight;
        unsigned int lcm_degree;
        int ret;
        int probe_ret;

        pr_info("%s+ jdi_probe enter p_d_a0031, vdo,120hz\n", __func__);

        dsi_node = of_get_parent(dev->of_node);
        if (dsi_node) {
                endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
                if (endpoint) {
                        remote_node = of_graph_get_remote_port_parent(endpoint);
                        if (!remote_node) {
                                pr_info("No panel connected,skip probe lcm\n");
                                return -ENODEV;
                        }
                        pr_info("device node name:%s\n", remote_node->name);
                }
        }
        if (remote_node != dev->of_node) {
                pr_info("%s+ skip probe due to not current lcm\n", __func__);
                return -ENODEV;
        }

        ctx = devm_kzalloc(dev, sizeof(struct jdi), GFP_KERNEL);
        if (!ctx)
                return -ENOMEM;

        mipi_dsi_set_drvdata(dsi, ctx);

        ctx->dev = dev;
        dsi->lanes = 4;
        dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
                        MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

        backlight = of_parse_phandle(dev->of_node, "backlight", 0);
        if (backlight) {
                ctx->backlight = of_find_backlight_by_node(backlight);
                of_node_put(backlight);

                if (!ctx->backlight)
                        return -EPROBE_DEFER;
        }

        ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->reset_gpio)) {
                dev_info(dev, "cannot get reset-gpio %ld\n",
                         PTR_ERR(ctx->reset_gpio));
                return PTR_ERR(ctx->reset_gpio);
        }
        devm_gpiod_put(dev, ctx->reset_gpio);

        ctx->tpreset_gpio = devm_gpiod_get(dev, "tpreset", GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->tpreset_gpio)) {
                dev_info(dev, "cannot get tpreset_gpio %ld\n",
                         PTR_ERR(ctx->tpreset_gpio));
                return PTR_ERR(ctx->tpreset_gpio);
        }
        devm_gpiod_put(dev, ctx->tpreset_gpio);

        ctx->bias_pos = devm_gpiod_get_index(dev, "bias", 0, GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->bias_pos)) {
                dev_info(dev, "cannot get bias-gpios 0 %ld\n",
                PTR_ERR(ctx->bias_pos));
                return PTR_ERR(ctx->bias_pos);
        }
        devm_gpiod_put(dev, ctx->bias_pos);

        ctx->bias_neg = devm_gpiod_get_index(dev, "bias", 1, GPIOD_OUT_HIGH);
        if (IS_ERR(ctx->bias_neg)) {
                dev_info(dev, "cannot get bias-gpios 1 %ld\n",
                PTR_ERR(ctx->bias_neg));
                return PTR_ERR(ctx->bias_neg);
        }
        devm_gpiod_put(dev, ctx->bias_neg);

	ctx->esd_te_gpio = devm_gpiod_get(ctx->dev, "esd-te", GPIOD_IN);
	if (IS_ERR(ctx->esd_te_gpio)) {
		dev_info(dev, "cannot get esd-te-gpios %ld\n",
			PTR_ERR(ctx->esd_te_gpio));
		return PTR_ERR(ctx->esd_te_gpio);
	}
	gpiod_direction_input(ctx->esd_te_gpio);

        ctx->prepared = true;
        ctx->enabled = true;
        drm_panel_init(&ctx->panel, dev, &jdi_drm_funcs, DRM_MODE_CONNECTOR_DSI);

        drm_panel_add(&ctx->panel);

        ret = mipi_dsi_attach(dsi);
        if (ret < 0)
                drm_panel_remove(&ctx->panel);

        /* check_is_bdg_support(dev); */
#if defined(CONFIG_MTK_PANEL_EXT)
        mtk_panel_tch_handle_reg(&ctx->panel);
        ret = mtk_panel_ext_create(dev, &ext_params_120hz, &ext_funcs, &ctx->panel);
        if (ret < 0)
                return ret;
        probe_ret = of_property_read_u32(dev->of_node, "lcm-degree", &lcm_degree);
        if (probe_ret < 0)
                lcm_degree = 0;
        else
                ext_params_120hz.lcm_degree = lcm_degree;
        pr_info("lcm_degree: %d\n", ext_params_120hz.lcm_degree);
#endif
	pr_info(" %s+ jdi_probe exit \n", __func__);
	register_device_proc("lcd", "A0031", "P_D");

	ctx->lcd_vsn_reset_nb = lcd_vsn_reset_notifier;
	register_reboot_notifier(&ctx->lcd_vsn_reset_nb);

	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	mipi_volt = MIPI_DRIVER_VOLTAGE;
	return ret;
}

static int jdi_remove(struct mipi_dsi_device *dsi)
{
        struct jdi *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
        struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

        if (ext_ctx == NULL) {
                return 0;
        }
        mipi_dsi_detach(dsi);
        drm_panel_remove(&ctx->panel);
#if defined(CONFIG_MTK_PANEL_EXT)
        mtk_panel_detach(ext_ctx);
        mtk_panel_remove(ext_ctx);
#endif

        unregister_reboot_notifier(&ctx->lcd_vsn_reset_nb);

        return 0;
}

static const struct of_device_id jdi_of_match[] = {
        {
                .compatible = "ae152,p_d,a0031,hd,dsi,vdo",
        },
        {
        }
};

MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_driver = {
        .probe = jdi_probe,
        .remove = jdi_remove,
        .driver = {
                .name = "ae152_p_d_a0031_hd_dsi_vdo",
                .owner = THIS_MODULE,
                .of_match_table = jdi_of_match,
        },
};

module_mipi_dsi_driver(jdi_driver);

MODULE_AUTHOR("shaohua deng <shaohua.deng@mediatek.com>");
MODULE_DESCRIPTION("P_D A0031 VDO 120HZ LCD Panel Driver");
MODULE_LICENSE("GPL v2");
