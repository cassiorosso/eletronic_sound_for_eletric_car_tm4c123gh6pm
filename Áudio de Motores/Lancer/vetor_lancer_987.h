/*
Mitsubishi Lancer a 987 RPM
Frequ�ncia de Amostragem:
N�mero de Amostras

void play_lancer_987(void)
{
  for(i = 0; i < 3233; i++)
  {
  Send_To_DAC8411(lancer_987[i],tempo);
  }  
}

void tempo_lancer_987(void)
{
tempo = ((-0.0015*rpm) + 28.13);
}

*/

const uint16_t lancer_987[3233]={
   33106,
   33568,
   33952,
   34322,
   34817,
   35364,
   35822,
   36183,
   36393,
   36499,
   36530,
   36579,
   36628,
   36775,
   36965,
   37152,
   37365,
   37650,
   38011,
   38421,
   38804,
   39126,
   39515,
   39919,
   40260,
   40591,
   40947,
   41320,
   41749,
   42244,
   42694,
   43148,
   43630,
   44091,
   44549,
   44992,
   45430,
   45886,
   46366,
   46749,
   47083,
   47468,
   47908,
   48336,
   48734,
   49065,
   49400,
   49734,
   50044,
   50280,
   50468,
   50748,
   51119,
   51543,
   51895,
   52215,
   52470,
   52660,
   52822,
   52980,
   53105,
   53227,
   53300,
   53325,
   53307,
   53277,
   53191,
   53076,
   52924,
   52732,
   52546,
   52362,
   52148,
   51921,
   51717,
   51500,
   51330,
   51234,
   51145,
   51025,
   50862,
   50677,
   50430,
   50116,
   49749,
   49350,
   48978,
   48625,
   48338,
   48052,
   47692,
   47353,
   47054,
   46789,
   46498,
   46255,
   46021,
   45853,
   45762,
   45738,
   45624,
   45393,
   45095,
   44816,
   44540,
   44271,
   43963,
   43648,
   43314,
   43002,
   42660,
   42337,
   42004,
   41734,
   41589,
   41504,
   41453,
   41449,
   41546,
   41661,
   41871,
   42162,
   42428,
   42625,
   42793,
   42933,
   42998,
   43046,
   43071,
   43062,
   43097,
   43231,
   43334,
   43334,
   43257,
   43239,
   43292,
   43408,
   43473,
   43474,
   43462,
   43433,
   43383,
   43306,
   43200,
   43088,
   42982,
   42859,
   42673,
   42408,
   42086,
   41708,
   41319,
   41016,
   40719,
   40401,
   40021,
   39627,
   39196,
   38769,
   38362,
   37953,
   37567,
   37184,
   36866,
   36553,
   36236,
   35836,
   35388,
   34905,
   34506,
   34147,
   33790,
   33399,
   33015,
   32691,
   32398,
   32091,
   31766,
   31372,
   30947,
   30519,
   30076,
   29645,
   29225,
   28877,
   28555,
   28268,
   27963,
   27647,
   27320,
   27050,
   26827,
   26664,
   26571,
   26567,
   26660,
   26816,
   26996,
   27150,
   27286,
   27327,
   27261,
   27127,
   26993,
   26866,
   26782,
   26700,
   26604,
   26461,
   26275,
   26029,
   25748,
   25502,
   25251,
   25012,
   24721,
   24500,
   24305,
   24180,
   24065,
   23978,
   23895,
   23807,
   23674,
   23521,
   23364,
   23201,
   23021,
   22878,
   22841,
   22823,
   22816,
   22776,
   22734,
   22704,
   22716,
   22680,
   22638,
   22609,
   22593,
   22560,
   22458,
   22325,
   22179,
   22009,
   21788,
   21560,
   21389,
   21284,
   21205,
   21122,
   21065,
   21044,
   21034,
   21045,
   21064,
   21159,
   21307,
   21472,
   21636,
   21813,
   22001,
   22191,
   22344,
   22566,
   22828,
   23122,
   23393,
   23669,
   23883,
   24065,
   24259,
   24525,
   24808,
   25083,
   25349,
   25573,
   25744,
   25885,
   26012,
   26111,
   26193,
   26338,
   26521,
   26671,
   26773,
   26854,
   26953,
   27078,
   27225,
   27373,
   27476,
   27528,
   27561,
   27590,
   27656,
   27766,
   27961,
   28170,
   28357,
   28539,
   28700,
   28831,
   28894,
   28980,
   29132,
   29305,
   29508,
   29681,
   29783,
   29841,
   29914,
   30035,
   30098,
   30174,
   30352,
   30542,
   30653,
   30730,
   30893,
   31087,
   31231,
   31324,
   31383,
   31430,
   31482,
   31535,
   31539,
   31480,
   31425,
   31389,
   31370,
   31403,
   31496,
   31652,
   31885,
   32198,
   32540,
   32883,
   33237,
   33618,
   33976,
   34288,
   34534,
   34734,
   34879,
   34948,
   34938,
   34933,
   34946,
   34930,
   34863,
   34828,
   34782,
   34643,
   34447,
   34273,
   34072,
   33939,
   33760,
   33594,
   33453,
   33316,
   33212,
   33114,
   33014,
   32939,
   32928,
   32945,
   32959,
   32961,
   32955,
   32937,
   32915,
   32903,
   32930,
   32954,
   32984,
   32962,
   32916,
   32845,
   32730,
   32619,
   32539,
   32546,
   32638,
   32727,
   32736,
   32662,
   32586,
   32544,
   32488,
   32394,
   32335,
   32332,
   32350,
   32366,
   32353,
   32333,
   32283,
   32221,
   32133,
   32039,
   31929,
   31773,
   31596,
   31487,
   31424,
   31363,
   31289,
   31221,
   31163,
   31101,
   31063,
   31011,
   30948,
   30912,
   30955,
   30957,
   30916,
   30860,
   30833,
   30777,
   30740,
   30765,
   30793,
   30735,
   30587,
   30439,
   30337,
   30325,
   30395,
   30477,
   30570,
   30708,
   30856,
   30940,
   30969,
   31081,
   31370,
   31827,
   32332,
   32819,
   33290,
   33761,
   34230,
   34669,
   35113,
   35623,
   36209,
   36841,
   37459,
   38023,
   38532,
   39043,
   39616,
   40247,
   40943,
   41660,
   42387,
   43096,
   43744,
   44340,
   44859,
   45357,
   45836,
   46306,
   46731,
   47102,
   47432,
   47746,
   48045,
   48347,
   48622,
   48929,
   49227,
   49536,
   49830,
   50108,
   50382,
   50643,
   50881,
   51113,
   51291,
   51409,
   51424,
   51396,
   51294,
   51098,
   50820,
   50464,
   50101,
   49757,
   49446,
   49129,
   48848,
   48583,
   48328,
   48048,
   47718,
   47410,
   47112,
   46829,
   46453,
   45949,
   45346,
   44658,
   43903,
   43082,
   42242,
   41432,
   40664,
   39883,
   39039,
   38164,
   37278,
   36392,
   35490,
   34598,
   33714,
   32822,
   31857,
   30795,
   29670,
   28587,
   27541,
   26462,
   25390,
   24351,
   23396,
   22456,
   21527,
   20583,
   19669,
   18849,
   18110,
   17345,
   16510,
   15648,
   14803,
   13982,
   13159,
   12355,
   11602,
   10944,
   10333,
    9761,
    9248,
    8822,
    8444,
    8115,
    7866,
    7683,
    7532,
    7412,
    7336,
    7275,
    7211,
    7133,
    7057,
    7022,
    6981,
    6918,
    6843,
    6825,
    6833,
    6821,
    6845,
    6859,
    6864,
    6873,
    6949,
    7025,
    7099,
    7190,
    7327,
    7488,
    7666,
    7909,
    8165,
    8475,
    8826,
    9231,
    9691,
   10276,
   10951,
   11699,
   12483,
   13335,
   14228,
   15188,
   16173,
   17180,
   18259,
   19449,
   20705,
   21948,
   23169,
   24351,
   25573,
   26831,
   28071,
   29321,
   30679,
   32185,
   33652,
   35084,
   36483,
   37814,
   39023,
   40197,
   41317,
   42452,
   43565,
   44569,
   45598,
   46604,
   47513,
   48372,
   49206,
   50061,
   50936,
   51778,
   52529,
   53082,
   53541,
   54035,
   54528,
   54932,
   55280,
   55659,
   56065,
   56378,
   56640,
   56846,
   57004,
   57175,
   57415,
   57661,
   57886,
   58125,
   58352,
   58498,
   58549,
   58598,
   58630,
   58643,
   58573,
   58429,
   58304,
   58210,
   58128,
   58020,
   57946,
   57932,
   57914,
   57855,
   57778,
   57696,
   57672,
   57712,
   57786,
   57844,
   57851,
   57833,
   57804,
   57852,
   57902,
   57940,
   57961,
   58024,
   58110,
   58201,
   58286,
   58356,
   58409,
   58505,
   58609,
   58674,
   58726,
   58787,
   58821,
   58806,
   58742,
   58635,
   58461,
   58238,
   57979,
   57636,
   57220,
   56668,
   55963,
   55077,
   54033,
   52944,
   51836,
   50711,
   49455,
   48045,
   46557,
   45010,
   43288,
   41283,
   39060,
   36825,
   34706,
   32668,
   30529,
   28268,
   26035,
   23989,
   22133,
   20374,
   18639,
   16985,
   15491,
   14181,
   12934,
   11753,
   10674,
    9805,
    9093,
    8524,
    8053,
    7671,
    7365,
    7128,
    6983,
    6915,
    6959,
    7077,
    7247,
    7453,
    7671,
    7924,
    8149,
    8401,
    8626,
    8847,
    9091,
    9319,
    9563,
    9808,
   10012,
   10169,
   10375,
   10634,
   10869,
   11003,
   11087,
   11166,
   11264,
   11283,
   11204,
   11053,
   10900,
   10722,
   10518,
   10267,
    9998,
    9731,
    9453,
    9188,
    8897,
    8592,
    8291,
    8024,
    7783,
    7567,
    7407,
    7305,
    7198,
    7065,
    6943,
    6869,
    6858,
    6884,
    6908,
    6995,
    7161,
    7442,
    7801,
    8240,
    8786,
    9414,
   10071,
   10786,
   11570,
   12438,
   13289,
   14103,
   14862,
   15658,
   16531,
   17517,
   18610,
   19781,
   20963,
   22113,
   23281,
   24501,
   25662,
   26721,
   27761,
   28904,
   30093,
   31206,
   32201,
   33114,
   33996,
   34848,
   35654,
   36392,
   37151,
   37985,
   38877,
   39737,
   40554,
   41357,
   42183,
   43099,
   44037,
   44891,
   45544,
   46114,
   46673,
   47291,
   47816,
   48233,
   48561,
   48971,
   49434,
   49866,
   50206,
   50549,
   50917,
   51267,
   51599,
   51947,
   52290,
   52594,
   52873,
   53111,
   53322,
   53505,
   53691,
   53835,
   53918,
   54005,
   54082,
   54117,
   54125,
   54156,
   54222,
   54289,
   54380,
   54501,
   54631,
   54803,
   54942,
   55057,
   55114,
   55156,
   55187,
   55190,
   55159,
   55075,
   55005,
   54851,
   54592,
   54308,
   53996,
   53734,
   53508,
   53285,
   53069,
   52906,
   52724,
   52525,
   52314,
   52131,
   51946,
   51777,
   51616,
   51394,
   51136,
   50858,
   50578,
   50244,
   49906,
   49556,
   49218,
   48916,
   48687,
   48514,
   48265,
   47924,
   47576,
   47325,
   47150,
   46910,
   46565,
   46231,
   46009,
   45833,
   45604,
   45319,
   45029,
   44724,
   44474,
   44327,
   44196,
   44004,
   43721,
   43456,
   43196,
   42941,
   42646,
   42312,
   41978,
   41746,
   41542,
   41295,
   40945,
   40629,
   40431,
   40367,
   40321,
   40251,
   40126,
   40079,
   40128,
   40230,
   40291,
   40321,
   40375,
   40500,
   40622,
   40660,
   40614,
   40624,
   40711,
   40771,
   40733,
   40688,
   40686,
   40665,
   40555,
   40401,
   40307,
   40315,
   40428,
   40555,
   40640,
   40711,
   40829,
   40955,
   41006,
   40947,
   40878,
   40902,
   40973,
   40974,
   40814,
   40631,
   40515,
   40474,
   40428,
   40358,
   40291,
   40227,
   40159,
   40060,
   39941,
   39837,
   39771,
   39717,
   39658,
   39585,
   39517,
   39408,
   39223,
   39015,
   38851,
   38718,
   38537,
   38284,
   37970,
   37697,
   37396,
   37100,
   36807,
   36584,
   36412,
   36192,
   35815,
   35452,
   35145,
   34830,
   34449,
   33997,
   33583,
   33290,
   33071,
   32794,
   32460,
   32186,
   31997,
   31750,
   31405,
   31007,
   30681,
   30382,
   30078,
   29734,
   29384,
   28978,
   28521,
   28045,
   27580,
   27143,
   26725,
   26364,
   26068,
   25851,
   25639,
   25420,
   25210,
   25081,
   25063,
   25062,
   25006,
   24898,
   24806,
   24729,
   24570,
   24326,
   24071,
   23888,
   23773,
   23644,
   23462,
   23268,
   23094,
   23016,
   22986,
   22979,
   22968,
   22985,
   22986,
   22944,
   22820,
   22679,
   22555,
   22438,
   22327,
   22204,
   22043,
   21832,
   21630,
   21526,
   21565,
   21684,
   21872,
   22092,
   22317,
   22449,
   22493,
   22481,
   22472,
   22429,
   22369,
   22313,
   22282,
   22228,
   22114,
   21991,
   21923,
   21954,
   22053,
   22173,
   22345,
   22552,
   22785,
   22965,
   23144,
   23351,
   23616,
   23845,
   24029,
   24185,
   24361,
   24516,
   24627,
   24750,
   24972,
   25240,
   25491,
   25665,
   25814,
   25985,
   26176,
   26400,
   26661,
   26986,
   27363,
   27711,
   28014,
   28203,
   28352,
   28470,
   28613,
   28767,
   28943,
   29136,
   29336,
   29518,
   29668,
   29808,
   29978,
   30195,
   30441,
   30678,
   30866,
   31028,
   31158,
   31258,
   31343,
   31386,
   31436,
   31415,
   31352,
   31189,
   30943,
   30645,
   30428,
   30331,
   30358,
   30450,
   30588,
   30772,
   30943,
   31050,
   31109,
   31198,
   31361,
   31579,
   31765,
   31891,
   31989,
   32036,
   32054,
   32031,
   32041,
   32091,
   32165,
   32232,
   32297,
   32345,
   32427,
   32567,
   32788,
   33039,
   33300,
   33523,
   33696,
   33805,
   33871,
   33916,
   33947,
   33983,
   33978,
   33965,
   33918,
   33875,
   33830,
   33799,
   33801,
   33819,
   33781,
   33708,
   33611,
   33538,
   33500,
   33448,
   33435,
   33391,
   33345,
   33235,
   33094,
   32934,
   32764,
   32606,
   32438,
   32252,
   32005,
   31699,
   31422,
   31199,
   31026,
   30831,
   30645,
   30466,
   30325,
   30168,
   29978,
   29794,
   29648,
   29551,
   29449,
   29313,
   29132,
   28930,
   28704,
   28530,
   28424,
   28326,
   28207,
   28111,
   28113,
   28178,
   28256,
   28310,
   28387,
   28511,
   28682,
   28814,
   28824,
   28748,
   28665,
   28611,
   28574,
   28561,
   28591,
   28629,
   28731,
   28897,
   29101,
   29291,
   29490,
   29752,
   30107,
   30501,
   30894,
   31171,
   31359,
   31570,
   31846,
   32086,
   32236,
   32378,
   32602,
   32876,
   33144,
   33358,
   33566,
   33818,
   34103,
   34364,
   34597,
   34893,
   35253,
   35600,
   35891,
   36203,
   36546,
   36798,
   36940,
   37035,
   37149,
   37285,
   37436,
   37578,
   37740,
   37967,
   38249,
   38512,
   38757,
   39024,
   39333,
   39657,
   39922,
   40175,
   40370,
   40472,
   40491,
   40433,
   40332,
   40193,
   40067,
   39964,
   39873,
   39787,
   39654,
   39478,
   39301,
   39149,
   39033,
   38996,
   38977,
   38886,
   38754,
   38643,
   38502,
   38354,
   38263,
   38171,
   37981,
   37690,
   37343,
   36921,
   36437,
   35953,
   35517,
   35139,
   34748,
   34277,
   33703,
   33058,
   32353,
   31600,
   30800,
   30024,
   29266,
   28473,
   27670,
   26907,
   26120,
   25230,
   24362,
   23546,
   22734,
   21972,
   21307,
   20644,
   19967,
   19402,
   18903,
   18375,
   17846,
   17426,
   17052,
   16686,
   16254,
   15848,
   15447,
   15133,
   14849,
   14625,
   14406,
   14169,
   14014,
   13947,
   13917,
   13885,
   13886,
   13937,
   14031,
   14172,
   14323,
   14455,
   14580,
   14678,
   14779,
   14932,
   15107,
   15344,
   15696,
   16154,
   16711,
   17386,
   18119,
   18917,
   19823,
   20772,
   21638,
   22577,
   23680,
   24809,
   25917,
   27044,
   28162,
   29259,
   30409,
   31497,
   32639,
   33923,
   35303,
   36675,
   38067,
   39404,
   40693,
   41965,
   42997,
   43514,
   44025,
   44923,
   45884,
   46601,
   47175,
   47661,
   48072,
   48616,
   49262,
   49847,
   50429,
   51139,
   51824,
   52432,
   52969,
   53422,
   53840,
   54305,
   54761,
   55150,
   55528,
   55879,
   56131,
   56329,
   56464,
   56529,
   56573,
   56588,
   56569,
   56517,
   56467,
   56400,
   56384,
   56437,
   56482,
   56532,
   56585,
   56578,
   56460,
   56337,
   56209,
   56003,
   55769,
   55596,
   55479,
   55345,
   55170,
   54953,
   54691,
   54437,
   54236,
   54058,
   53829,
   53625,
   53440,
   53238,
   52987,
   52745,
   52523,
   52259,
   51873,
   51382,
   50812,
   50151,
   49319,
   48380,
   47399,
   46472,
   45537,
   44538,
   43545,
   42711,
   42088,
   41458,
   40760,
   40225,
   39937,
   39745,
   39431,
   38986,
   38442,
   37894,
   37402,
   36966,
   36480,
   35893,
   35188,
   34592,
   34230,
   33971,
   33645,
   33341,
   33206,
   33102,
   32931,
   32663,
   32363,
   32037,
   31820,
   31701,
   31623,
   31431,
   31222,
   31167,
   31260,
   31292,
   31108,
   30826,
   30489,
   30083,
   29553,
   28986,
   28412,
   27859,
   27275,
   26724,
   26173,
   25581,
   24832,
   23992,
   23304,
   22839,
   22302,
   21565,
   20866,
   20392,
   19964,
   19330,
   18556,
   17803,
   17125,
   16505,
   15855,
   15181,
   14513,
   13892,
   13299,
   12721,
   12171,
   11716,
   11318,
   10877,
   10397,
   10009,
    9683,
    9352,
    9007,
    8783,
    8651,
    8509,
    8351,
    8191,
    8117,
    8096,
    8073,
    8018,
    7969,
    7970,
    7958,
    7956,
    7961,
    7978,
    7982,
    8028,
    8062,
    8094,
    8171,
    8279,
    8381,
    8510,
    8696,
    8905,
    9146,
    9468,
    9852,
   10216,
   10556,
   10947,
   11262,
   11509,
   11748,
   12035,
   12323,
   12613,
   12957,
   13320,
   13700,
   14162,
   14714,
   15249,
   15744,
   16292,
   16917,
   17549,
   18163,
   18783,
   19438,
   20035,
   20565,
   21074,
   21490,
   21712,
   21893,
   22199,
   22520,
   22726,
   22940,
   23231,
   23533,
   23839,
   24173,
   24442,
   24729,
   25095,
   25466,
   25774,
   26147,
   26563,
   26961,
   27398,
   27914,
   28425,
   28899,
   29358,
   29767,
   30071,
   30295,
   30505,
   30790,
   31158,
   31462,
   31702,
   31979,
   32307,
   32652,
   33097,
   33635,
   34135,
   34633,
   35146,
   35613,
   35997,
   36491,
   37096,
   37684,
   38244,
   38796,
   39332,
   39887,
   40508,
   41128,
   41793,
   42538,
   43321,
   44074,
   44800,
   45470,
   46109,
   46783,
   47437,
   48033,
   48576,
   49120,
   49587,
   49923,
   50191,
   50421,
   50644,
   50836,
   50971,
   51131,
   51266,
   51306,
   51326,
   51411,
   51486,
   51507,
   51503,
   51452,
   51355,
   51265,
   51133,
   51029,
   50994,
   51007,
   50928,
   50804,
   50665,
   50533,
   50398,
   50232,
   50031,
   49828,
   49661,
   49427,
   49183,
   48900,
   48574,
   48238,
   47964,
   47717,
   47484,
   47282,
   46999,
   46683,
   46366,
   46062,
   45772,
   45481,
   45203,
   44913,
   44729,
   44677,
   44678,
   44649,
   44593,
   44581,
   44691,
   44801,
   44851,
   44854,
   44912,
   44991,
   45109,
   45173,
   45268,
   45395,
   45442,
   45425,
   45535,
   45756,
   45933,
   46099,
   46351,
   46601,
   46804,
   47008,
   47171,
   47256,
   47311,
   47353,
   47312,
   47203,
   47035,
   46868,
   46696,
   46503,
   46360,
   46307,
   46275,
   46265,
   46329,
   46414,
   46473,
   46463,
   46440,
   46383,
   46346,
   46302,
   46222,
   46167,
   46096,
   45976,
   45755,
   45441,
   45082,
   44731,
   44331,
   43914,
   43623,
   43380,
   43065,
   42708,
   42367,
   41932,
   41487,
   41057,
   40629,
   40237,
   39897,
   39482,
   38940,
   38395,
   37811,
   37185,
   36553,
   36005,
   35489,
   35030,
   34578,
   34157,
   33787,
   33529,
   33372,
   33298,
   33242,
   33144,
   33092,
   33046,
   32969,
   32852,
   32798,
   32757,
   32694,
   32584,
   32435,
   32198,
   31874,
   31501,
   31113,
   30777,
   30477,
   30235,
   29936,
   29584,
   29252,
   28972,
   28692,
   28365,
   28009,
   27724,
   27473,
   27201,
   26823,
   26474,
   26181,
   25897,
   25629,
   25479,
   25403,
   25272,
   25059,
   24855,
   24673,
   24540,
   24382,
   24215,
   24044,
   23891,
   23685,
   23431,
   23180,
   22951,
   22649,
   22300,
   21966,
   21699,
   21438,
   21150,
   20873,
   20675,
   20571,
   20435,
   20278,
   20177,
   20114,
   20021,
   19960,
   19991,
   20037,
   20065,
   20130,
   20188,
   20224,
   20280,
   20347,
   20384,
   20452,
   20578,
   20671,
   20760,
   20901,
   21048,
   21188,
   21291,
   21403,
   21579,
   21772,
   21952,
   22108,
   22304,
   22464,
   22585,
   22709,
   22758,
   22718,
   22725,
   22810,
   22914,
   23064,
   23274,
   23469,
   23653,
   23813,
   23897,
   23998,
   24160,
   24350,
   24525,
   24667,
   24809,
   24944,
   25047,
   25116,
   25175,
   25269,
   25366,
   25481,
   25643,
   25781,
   25913,
   26145,
   26424,
   26731,
   27058,
   27386,
   27558,
   27682,
   27823,
   27920,
   27900,
   27888,
   27941,
   28021,
   28082,
   28164,
   28239,
   28324,
   28459,
   28713,
   29026,
   29347,
   29715,
   30120,
   30506,
   30841,
   31173,
   31497,
   31803,
   32108,
   32340,
   32550,
   32716,
   32866,
   32997,
   33139,
   33316,
   33519,
   33738,
   33908,
   34026,
   34157,
   34240,
   34281,
   34341,
   34454,
   34535,
   34550,
   34529,
   34473,
   34422,
   34374,
   34319,
   34240,
   34127,
   33976,
   33867,
   33779,
   33728,
   33688,
   33705,
   33736,
   33820,
   33975,
   34113,
   34215,
   34287,
   34361,
   34403,
   34397,
   34359,
   34340,
   34380,
   34433,
   34449,
   34479,
   34450,
   34409,
   34369,
   34317,
   34190,
   34113,
   34139,
   34194,
   34228,
   34244,
   34193,
   34123,
   34058,
   33970,
   33801,
   33658,
   33531,
   33305,
   33048,
   32779,
   32502,
   32216,
   31995,
   31806,
   31659,
   31514,
   31329,
   31144,
   30995,
   30877,
   30821,
   30829,
   30808,
   30772,
   30755,
   30653,
   30475,
   30326,
   30224,
   30093,
   30006,
   29955,
   29907,
   29867,
   29895,
   29977,
   30094,
   30222,
   30351,
   30456,
   30607,
   30764,
   30933,
   31062,
   31161,
   31314,
   31510,
   31678,
   31774,
   31870,
   32013,
   32178,
   32395,
   32672,
   32962,
   33253,
   33520,
   33785,
   34115,
   34478,
   34851,
   35222,
   35644,
   36071,
   36470,
   36857,
   37245,
   37628,
   38007,
   38402,
   38801,
   39148,
   39507,
   39860,
   40208,
   40577,
   41004,
   41438,
   41813,
   42133,
   42446,
   42745,
   43027,
   43294,
   43516,
   43737,
   43944,
   44138,
   44329,
   44486,
   44636,
   44755,
   44907,
   45062,
   45179,
   45266,
   45344,
   45430,
   45548,
   45650,
   45765,
   45857,
   45980,
   46083,
   46161,
   46195,
   46207,
   46266,
   46371,
   46458,
   46532,
   46562,
   46599,
   46586,
   46544,
   46465,
   46346,
   46224,
   46066,
   45896,
   45623,
   45234,
   44798,
   44439,
   44129,
   43788,
   43416,
   43023,
   42580,
   42033,
   41426,
   40794,
   40130,
   39444,
   38741,
   38011,
   37249,
   36415,
   35470,
   34439,
   33384,
   32365,
   31333,
   30325,
   29362,
   28397,
   27354,
   26281,
   25184,
   24084,
   22995,
   21943,
   20926,
   19959,
   19064,
   18191,
   17340,
   16435,
   15542,
   14713,
   13887,
   13128,
   12478,
   11877,
   11314,
   10809,
   10342,
    9894,
    9494,
    9175,
    8863,
    8574,
    8339,
    8167,
    8036,
    7929,
    7834,
    7776,
    7744,
    7742,
    7737,
    7758,
    7798,
    7863,
    7904,
    7960,
    8027,
    8097,
    8145,
    8239,
    8349,
    8473,
    8591,
    8720,
    8872,
    9046,
    9265,
    9506,
    9810,
   10197,
   10660,
   11190,
   11791,
   12517,
   13367,
   14308,
   15300,
   16365,
   17507,
   18717,
   19938,
   21172,
   22417,
   23693,
   24961,
   26293,
   27685,
   29127,
   30556,
   32006,
   33516,
   35079,
   36624,
   38053,
   39386,
   40728,
   42163,
   43568,
   44774,
   45741,
   46559,
   47304,
   47891,
   48254,
   48502,
   48846,
   49354,
   49876,
   50298,
   50697,
   51164,
   51651,
   52121,
   52610,
   53138,
   53655,
   54152,
   54577,
   54922,
   55172,
   55392,
   55558,
   55700,
   55890,
   56137,
   56320,
   56394,
   56479,
   56633,
   56785,
   56903,
   57008,
   57087,
   57122,
   57169,
   57220,
   57220,
   57211,
   57280,
   57451,
   57601,
   57692,
   57733,
   57749,
   57767,
   57802,
   57818,
   57770,
   57740,
   57787,
   57867,
   57906,
   57935,
   57972,
   57995,
   57991,
   58024,
   58106,
   58155,
   58200,
   58227,
   58273,
   58322,
   58344,
   58333,
   58271,
   58239,
   58258,
   58277,
   58252,
   58159,
   58120,
   58057,
   57946,
   57708,
   57318,
   56885,
   56478,
   56078,
   55527,
   54850,
   54106,
   53364,
   52566,
   51653,
   50590,
   49527,
   48514,
   47448,
   46224,
   44774,
   43183,
   41510,
   39811,
   38150,
   36566,
   34974,
   33273,
   31413,
   29461,
   27510,
   25598,
   23695,
   21820,
   20152,
   18719,
   17415,
   16185,
   15070,
   14122,
   13261,
   12366,
   11484,
   10703,
    9997,
    9336,
    8806,
    8430,
    8099,
    7798,
    7613,
    7472,
    7297,
    7157,
    7112,
    7092,
    7076,
    7123,
    7208,
    7230,
    7256,
    7269,
    7299,
    7346,
    7412,
    7390,
    7367,
    7383,
    7420,
    7439,
    7480,
    7519,
    7583,
    7586,
    7611,
    7640,
    7636,
    7567,
    7493,
    7424,
    7285,
    7173,
    7090,
    7000,
    6904,
    6804,
    6760,
    6805,
    6873,
    6904,
    6981,
    7072,
    7156,
    7344,
    7564,
    7749,
    8000,
    8336,
    8571,
    8788,
    9017,
    9304,
    9715,
   10262,
   10796,
   11415,
   12129,
   12806,
   13437,
   14081,
   14716,
   15363,
   16083,
   16896,
   17771,
   18659,
   19438,
   20178,
   20818,
   21341,
   21767,
   22151,
   22594,
   23205,
   23894,
   24482,
   25216,
   25995,
   26529,
   26916,
   27548,
   28317,
   29091,
   30002,
   31065,
   32110,
   32909,
   33402,
   33765,
   34108,
   34382,
   34632,
   35001,
   35436,
   35849,
   36166,
   36387,
   36615,
   37001,
   37540,
   38172,
   38831,
   39476,
   40089,
   40649,
   41087,
   41514,
   42050,
   42657,
   43341,
   43967,
   44396,
   44641,
   44873,
   45102,
   45310,
   45471,
   45614,
   45765,
   45965,
   46065,
   46109,
   46303,
   46611,
   46942,
   47338,
   47833,
   48245,
   48442,
   48561,
   48699,
   48913,
   49094,
   49246,
   49385,
   49526,
   49592,
   49557,
   49478,
   49464,
   49546,
   49712,
   49849,
   49982,
   50121,
   50344,
   50636,
   50922,
   51147,
   51412,
   51715,
   51910,
   51993,
   52061,
   52137,
   52158,
   52215,
   52312,
   52389,
   52450,
   52497,
   52445,
   52376,
   52426,
   52476,
   52476,
   52460,
   52527,
   52526,
   52402,
   52252,
   52139,
   52098,
   52086,
   52027,
   51962,
   51927,
   51951,
   51835,
   51667,
   51552,
   51451,
   51276,
   51085,
   50889,
   50648,
   50468,
   50227,
   49791,
   49265,
   48831,
   48360,
   47904,
   47584,
   47321,
   47068,
   46833,
   46604,
   46317,
   45995,
   45625,
   45242,
   44969,
   44810,
   44673,
   44450,
   44144,
   43770,
   43442,
   43155,
   42925,
   42754,
   42580,
   42366,
   42252,
   42231,
   42252,
   42280,
   42382,
   42544,
   42693,
   42722,
   42675,
   42664,
   42686,
   42692,
   42700,
   42726,
   42646,
   42391,
   42054,
   41791,
   41631,
   41475,
   41340,
   41233,
   41081,
   40819,
   40463,
   40082,
   39685,
   39389,
   39184,
   39037,
   38838,
   38493,
   38028,
   37569,
   37205,
   36916,
   36692,
   36558,
   36428,
   36257,
   35989,
   35634,
   35321,
   35149,
   34992,
   34784,
   34533,
   34303,
   34048,
   33769,
   33425,
   33137,
   32938,
   32741,
   32497,
   32274,
   32021,
   31679,
   31375,
   31169,
   31002,
   30854,
   30732,
   30509,
   30229,
   29946,
   29631,
   29264,
   28878,
   28555,
   28237,
   27917,
   27576,
   27236,
   26907,
   26613,
   26380,
   26252,
   26131,
   25972,
   25806,
   25686,
   25532,
   25307,
   25076,
   24856,
   24688,
   24542,
   24369,
   24127,
   23885,
   23657,
   23457,
   23337,
   23362,
   23386,
   23343,
   23334,
   23367,
   23408,
   23387,
   23371,
   23417,
   23527,
   23614,
   23666,
   23736,
   23805,
   23894,
   23988,
   24049,
   24052,
   24054,
   24067,
   24102,
   24198,
   24426,
   24719,
   25003,
   25268,
   25467,
   25528,
   25536,
   25590,
   25698,
   25836,
   26009,
   26136,
   26123,
   26003,
   25815,
   25608,
   25496,
   25478,
   25499,
   25452,
   25347,
   25216,
   25130,
   25115,
   25157,
   25303,
   25547,
   25744,
   25783,
   25714,
   25592,
   25534,
   25595,
   25756,
   25919,
   26094,
   26268,
   26349,
   26352,
   26407,
   26575,
   26800,
   27054,
   27308,
   27534,
   27744,
   27916,
   28027,
   28118,
   28234,
   28344,
   28423,
   28438,
   28453,
   28454,
   28479,
   28485,
   28584,
   28772,
   28950,
   29029,
   29088,
   29161,
   29221,
   29254,
   29273,
   29304,
   29335,
   29328,
   29196,
   28954,
   28736,
   28576,
   28457,
   28308,
   28234,
   28184,
   28191,
   28250,
   28419,
   28693,
   29023,
   29372,
   29718,
   30022,
   30238,
   30452,
   30648,
   30845,
   31058,
   31294,
   31506,
   31659,
   31779,
   31855,
   31914,
   31956,
   32011,
   32107,
   32214,
   32283,
   32285,
   32321,
   32393,
   32464,
   32513,
   32574,
   32637,
   32699,
   32714,
   32690,
   32632,
   32597,
   32569,
   32567,
   32567,
   32562,
   32478,
   32383,
   32301,
   32203,
   32101,
   31998,
   31855,
   31738,
   31643,
   31475,
   31228,
   30971,
   30706,
   30402,
   30153,
   29965,
   29719,
   29460,
   29256,
   29144,
   29088,
   29085,
   29112,
   29111,
   29137,
   29110,
   29025,
   28892,
   28726,
   28625,
   28575,
   28481,
   28249,
   28002,
   27831,
   27685,
   27548,
   27513,
   27525,
   27589,
   27694,
   27808,
   27922,
   28124,
   28383,
   28623,
   28883,
   29171,
   29398,
   29532,
   29623,
   29667,
   29690,
   29741,
   29805,
   29820,
   29818,
   29794,
   29749,
   29721,
   29748,
   29836,
   30003,
   30253,
   30499,
   30695,
   30844,
   30978,
   31117,
   31329,
   31568,
   31811,
   32054,
   32291,
   32508,
   32729,
   32984,
   33282,
   33649,
   34091,
   34586,
   35072,
   35541,
   35961,
   36361,
   36761,
   37124,
   37453,
   37716,
   37955,
   38159,
   38303,
   38374,
   38396,
   38406,
   38443,
   38504,
   38514,
   38457,
   38436,
   38537,
   38730,
   38948,
   39171,
   39389,
   39644,
   39954,
   40263,
   40512,
   40757,
   41026,
   41240,
   41379,
   41488,
   41541,
   41504,
   41365,
   41147,
   40925,
   40753,
   40609,
   40450,
   40366,
   40351,
   40337,
   40282,
   40139,
   39968,
   39791,
   39722,
   39753,
   39749,
   39728,
   39679,
   39543,
   39322,
   39026,
   38710,
   38404,
   38122,
   37814,
   37513,
   37120,
   36584,
   35982,
   35453,
   34943,
   34371,
   33793,
   33152,
   32420,
   31556,
   30615,
   29606,
   28597,
   27614,
   26633,
   25662,
   24690,
   23742,
   22830,
   22002,
   21222,
   20511,
   19897,
   19297,
   18642,
   17959,
   17287,
   16608,
   15939,
   15363,
   14877,
   14501,
   14196,
   13936,
   13694,
   13473,
   13309,
   13208,
   13203,
   13227,
   13294,
   13389,
   13488,
   13561,
   13638,
   13756,
   13920,
   14095,
   14319,
   14591,
   14889,
   15206,
   15547,
   15956,
   16426,
   16938,
   17505,
   18127,
   18842,
   19635,
   20512,
   21391,
   22254,
   23102,
   23952,
   24828,
   25672,
   26498,
   27325,
   28224,
   29190,
   30187,
   31211,
   32253,
   33336,
   34470,
   35653,
   36868,
   38088,
   39268,
   40336,
   41229,
   41968,
   42559,
   43051,
   43537,
   44007,
   44536,
   45100,
   45717,
   46315,
   46941,
   47620,
   48365,
   49074,
   49701,
   50241,
   50768,
   51240,
   51617,
   51879,
   52138,
   52381,
   52607,
   52792,
   52986,
   53225,
   53503,
   53767,
   54008,
   54213,
   54465,
   54719,
   54861,
   55004,
   55151,
   55266,
   55406,
   55517,
   55566,
   55525,
   55433,
   55314,
   55144,
   54960,
   54720,
   54515,
   54409,
   54451,
   54556,
   54610,
   54611,
   54651,
   54749,
   54795,
   54742,
   54663,
   54606,
   54529,
   54352,
   54062,
   53706,
   53358,
   52980,
   52571,
   52096,
   51617,
   51108,
   50607,
   50084,
   49617,
   49215,
   48726,
   48075,
   47439,
   46997,
   46660,
   46229,
   45689,
   45126,
   44673,
   44220,
   43588,
   42824,
   42099,
   41473,
   40877,
   40338,
   39818,
   39295,
   38779,
   38316,
   37919,
   37488,
   36943,
   36481,
   36252,
   36030,
   35560,
   35081,
   34712,
   34371,
   33960,
   33553,
   33167,
   32796,
   32328,
   31867,
   31561,
   31360,
   30881,
   30125,
   29349,
   28715,
   28141,
   27518,
   26934,
   26558,
   26299,
   25898,
   25402,
   24943,
   24386,
   23679,
   23013,
   22432,
   21839,
   21190,
   20541,
   19868,
   19253,
   18670,
   18127,
   17643,
   17120,
   16552,
   15989,
   15451,
   14878,
   14306,
   13826,
   13410,
   13017,
   12548,
   12026,
   11573,
   11212,
   10776,
   10311,
    9910,
    9588,
    9267,
    8936,
    8648,
    8410,
    8215,
    8052,
    7916,
    7842,
    7776,
    7703,
    7607,
    7507,
    7450,
    7440,
    7471,
    7513,
    7513,
    7507,
    7514,
    7533,
    7551,
    7597,
    7681,
    7743,
    7834,
    7933,
    8023,
    8089,
    8183,
    8341,
    8558,
    8756,
    8984,
    9265,
    9615,
    9951,
   10324,
   10780,
   11270,
   11814,
   12418,
   13001,
   13517,
   14085,
   14666,
   15233,
   15835,
   16497,
   17169,
   17835,
   18572,
   19282,
   20024,
   20782,
   21565,
   22304,
   23108,
   23997,
   25038,
   26034,
   26871,
   27624,
   28317,
   28823,
   29216,
   29707,
   30187,
   30602,
   31174,
   31830,
   32306};