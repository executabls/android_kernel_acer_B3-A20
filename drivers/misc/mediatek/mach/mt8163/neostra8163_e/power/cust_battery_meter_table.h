#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
#define BAT_NTC_10 1
#define BAT_NTC_47 0
#define BAT_NTC_100 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900
#define RBAT_PULL_DOWN_R           30000
#endif
#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#define RBAT_PULL_DOWN_R           100000
#endif
#if (BAT_NTC_100 == 1)
#define RBAT_PULL_UP_R             24000
#define RBAT_PULL_DOWN_R           100000000
#endif
#define RBAT_PULL_UP_VOLT          1800


// ============================================================
// ENUM
// ============================================================

// ============================================================
// structure
// ============================================================

// ============================================================
// typedef
// ============================================================
typedef struct _BATTERY_PROFILE_STRUC
{
    kal_int32 percentage;
    kal_int32 voltage;
} BATTERY_PROFILE_STRUC, *BATTERY_PROFILE_STRUC_P;

typedef struct _R_PROFILE_STRUC
{
    kal_int32 resistance; // Ohm
    kal_int32 voltage;
} R_PROFILE_STRUC, *R_PROFILE_STRUC_P;

typedef enum
{
    T1_0C,
    T2_25C,
    T3_50C
} PROFILE_TEMPERATURE;

// ============================================================
// External Variables
// ============================================================

// ============================================================
// External function
// ============================================================

// ============================================================
// <DOD, Battery_Voltage> Table
// ============================================================
#if (BAT_NTC_10 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20 , 72818},
		{-15 , 56587},
		{-10 , 44369},
		{ -5 , 35084},
		{  0 , 27965},
		{  5 , 22459},
		{ 10 , 18166},
		{ 15 , 14794},
		{ 20 , 12126},
		{ 25 , 10000},
		{ 30 , 8294},
		{ 35 , 6918},
		{ 40 , 5800},
		{ 45 , 4887},
		{ 50 , 4138},
		{ 55 , 3520},
		{ 60 , 3007}	
	};
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    };
#endif

#if (BAT_NTC_100 == 1)
	BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20,1151037},
		{-15,846579},
		{-10,628988},
		{ -5,471632},
		{  0,357012},
		{  5,272500},
		{ 10,209710},
		{ 15,162651},
		{ 20,127080},
		{ 25,100000},
		{ 30,79222},
		{ 35,63167},
		{ 40,50677},
		{ 45,40904},
		{ 50,33195},
		{ 55,27091},
		{ 60,22224}
	};
#endif
// T0 -10C
BATTERY_PROFILE_STRUC battery_profile_t0[] =
{
	{0	    ,  4201},
	{2	    ,  4181},
	{3	    ,  4163},
	{5	    ,  4146},
	{6	    ,  4129},
	{8	    ,  4115},
	{10 	,  4102},
	{11 	,  4086},
	{13 	,  4072},
	{15 	,  4058},
	{16 	,  4045},
	{18 	,  4030},
	{19 	,  4019},
	{21 	,  4005},
	{23 	,  3992},
	{24 	,  3981},
	{26 	,  3969},
	{27 	,  3958},
	{29 	,  3948},
	{31 	,  3932},
	{32 	,  3923},
	{34 	,  3908},
	{35 	,  3896},
	{37 	,  3884},
	{39 	,  3875},
	{40 	,  3866},
	{42 	,  3855},
	{44 	,  3847},
	{45 	,  3837},
	{47 	,  3829},
	{48 	,  3823},
	{50 	,  3818},
	{52 	,  3811},
	{53 	,  3809},
	{55 	,  3802},
	{56 	,  3796},
	{58 	,  3793},
	{60 	,  3789},
	{61 	,  3785},
	{63 	,  3782},
	{65 	,  3779},
	{66 	,  3777},
	{68 	,  3775},
	{69 	,  3773},
	{71 	,  3771},
	{73 	,  3769},
	{74 	,  3767},
	{76 	,  3764},
	{77 	,  3762},
	{79 	,  3758},
	{81 	,  3752},
	{82 	,  3747},
	{84 	,  3740},
	{85 	,  3730},
	{87 	,  3721},
	{89 	,  3711},
	{90 	,  3700},
	{92 	,  3693},
	{94 	,  3685},
	{95 	,  3677},
	{97 	,  3673},
	{98 	,  3660},
	{100    ,  3500},
	{100    ,  3500},
	{100    ,  3500},
	{100    ,  3500} 
};      
        
// T1 0C 
BATTERY_PROFILE_STRUC battery_profile_t1[] =
{
	{0	   ,   4198},
	{2	   ,   4178},
	{3	   ,   4159},
	{5	   ,   4146},
	{6	   ,   4127},
	{8	   ,   4112},
	{9	   ,   4097},
	{11    ,   4084},
	{13    ,   4071},
	{14    ,   4056},
	{16    ,   4042},
	{17    ,   4030},
	{19    ,   4019},
	{20    ,   4004},
	{22    ,   3992},
	{23    ,   3982},
	{25    ,   3971},
	{27    ,   3959},
	{28    ,   3948},
	{30    ,   3939},
	{31    ,   3929},
	{33    ,   3918},
	{34    ,   3908},
	{36    ,   3897},
	{38    ,   3885},
	{39    ,   3872},
	{41    ,   3861},
	{42    ,   3850},
	{44    ,   3842},
	{45    ,   3832},
	{47    ,   3823},
	{48    ,   3819},
	{50    ,   3814},
	{52    ,   3806},
	{53    ,   3801},
	{55    ,   3796},
	{56    ,   3791},
	{58    ,   3788},
	{59    ,   3784},
	{61    ,   3780},
	{63    ,   3780},
	{64    ,   3775},
	{66    ,   3774},
	{67    ,   3772},
	{69    ,   3771},
	{70    ,   3769},
	{72    ,   3768},
	{73    ,   3766},
	{75    ,   3764},
	{77    ,   3762},
	{78    ,   3757},
	{80    ,   3752},
	{81    ,   3744},
	{83    ,   3734},
	{84    ,   3722},
	{86    ,   3709},
	{88    ,   3695},
	{89    ,   3686},
	{91    ,   3678},
	{92    ,   3674},
	{94    ,   3668},
	{95    ,   3660},
	{97    ,   3641},
	{98    ,   3573},
	{100   ,   3500},
	{100   ,   3500} 
};           

// T2 25C
BATTERY_PROFILE_STRUC battery_profile_t2[] =
{
	{0	  ,  4193},
	{2	  ,  4172},
	{3	  ,  4155},
	{5	  ,  4138},
	{6	  ,  4122},
	{8	  ,  4107},
	{9	  ,  4092},
	{11   ,  4078},
	{12   ,  4063},
	{14   ,  4050},
	{15   ,  4037},
	{17   ,  4024},
	{18   ,  4011},
	{20   ,  3998},
	{22   ,  3987},
	{23   ,  3976},
	{25   ,  3965},
	{26   ,  3954},
	{28   ,  3945},
	{29   ,  3934},
	{31   ,  3926},
	{32   ,  3916},
	{34   ,  3907},
	{35   ,  3898},
	{37   ,  3889},
	{38   ,  3881},
	{40   ,  3869},
	{42   ,  3858},
	{43   ,  3846},
	{45   ,  3834},
	{46   ,  3823},
	{48   ,  3814},
	{49   ,  3807},
	{51   ,  3800},
	{52   ,  3794},
	{54   ,  3789},
	{55   ,  3785},
	{57   ,  3780},
	{58   ,  3776},
	{60   ,  3772},
	{62   ,  3770},
	{63   ,  3768},
	{65   ,  3764},
	{66   ,  3763},
	{68   ,  3759},
	{69   ,  3758},
	{71   ,  3756},
	{72   ,  3753},
	{74   ,  3752},
	{75   ,  3747},
	{77   ,  3742},
	{78   ,  3733},
	{80   ,  3726},
	{82   ,  3720},
	{83   ,  3710},
	{85   ,  3696},
	{86   ,  3683},
	{88   ,  3669},
	{89   ,  3664},
	{91   ,  3660},
	{92   ,  3655},
	{94   ,  3649},
	{95   ,  3634},
	{97   ,  3578},
	{98   ,  3512},
	{100  ,  3500}
};     

// T3 50C
BATTERY_PROFILE_STRUC battery_profile_t3[] =
{
	{0	  ,   4191},
	{2	  ,   4171},
	{3	  ,   4156},
	{5	  ,   4138},
	{6	  ,   4122},
	{8	  ,   4107},
	{9	  ,   4091},
	{11   ,   4077},
	{13   ,   4063},
	{14   ,   4048},
	{16   ,   4036},
	{17   ,   4023},
	{19   ,   4009},
	{20   ,   3998},
	{22   ,   3986},
	{23   ,   3974},
	{25   ,   3966},
	{27   ,   3954},
	{28   ,   3942},
	{30   ,   3933},
	{31   ,   3922},
	{33   ,   3913},
	{34   ,   3907},
	{36   ,   3898},
	{38   ,   3887},
	{39   ,   3881},
	{41   ,   3871},
	{42   ,   3860},
	{44   ,   3848},
	{45   ,   3834},
	{47   ,   3824},
	{48   ,   3815},
	{50   ,   3807},
	{52   ,   3803},
	{53   ,   3797},
	{55   ,   3792},
	{56   ,   3785},
	{58   ,   3782},
	{59   ,   3777},
	{61   ,   3773},
	{63   ,   3769},
	{64   ,   3767},
	{66   ,   3764},
	{67   ,   3763},
	{69   ,   3759},
	{70   ,   3756},
	{72   ,   3749},
	{73   ,   3742},
	{75   ,   3736},
	{77   ,   3730},
	{78   ,   3727},
	{80   ,   3716},
	{81   ,   3712},
	{83   ,   3703},
	{84   ,   3691},
	{86   ,   3677},
	{88   ,   3665},
	{89   ,   3656},
	{91   ,   3652},
	{92   ,   3648},
	{94   ,   3643},
	{95   ,   3633},
	{97   ,   3594},
	{98   ,   3513},
	{100  ,   3500},
	{100  ,   3500}
};           

// battery profile for actual temperature. The size should be the same as T1, T2 and T3
BATTERY_PROFILE_STRUC battery_profile_temperature[] =
{
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0}	
};    

// ============================================================
// <Rbat, Battery_Voltage> Table
// ============================================================
// T0 -10C
R_PROFILE_STRUC r_profile_t0[] =
{
	{160	,  4201},
	{160	,  4181},
	{159	,  4163},
	{161	,  4146},
	{162	,  4129},
	{163	,  4115},
	{170	,  4102},
	{170	,  4086},
	{172	,  4072},
	{175	,  4058},
	{178	,  4045},
	{181	,  4030},
	{187	,  4019},
	{188	,  4005},
	{191	,  3992},
	{190	,  3981},
	{194	,  3969},
	{195	,  3958},
	{195	,  3948},
	{191	,  3932},
	{192	,  3923},
	{189	,  3908},
	{184	,  3896},
	{181	,  3884},
	{177	,  3875},
	{178	,  3866},
	{169	,  3855},
	{170	,  3847},
	{168	,  3837},
	{166	,  3829},
	{170	,  3823},
	{166	,  3818},
	{168	,  3811},
	{175	,  3809},
	{169	,  3802},
	{170	,  3796},
	{172	,  3793},
	{174	,  3789},
	{173	,  3785},
	{175	,  3782},
	{174	,  3779},
	{177	,  3777},
	{178	,  3775},
	{176	,  3773},
	{179	,  3771},
	{181	,  3769},
	{181	,  3767},
	{185	,  3764},
	{186	,  3762},
	{187	,  3758},
	{186	,  3752},
	{188	,  3747},
	{190	,  3740},
	{187	,  3730},
	{192	,  3721},
	{193	,  3711},
	{192	,  3700},
	{200	,  3693},
	{206	,  3685},
	{226	,  3677},
	{269	,  3673},
	{345	,  3660},
	{519    ,  3500}, 
	{519    ,  3500}, 
	{519    ,  3500}, 
	{519	,  3500} 
};    
// T1 0C
R_PROFILE_STRUC r_profile_t1[] =
{
	{105   ,   4198},
	{105   ,   4178},
	{103   ,   4159},
	{108   ,   4146},
	{105   ,   4127},
	{108   ,   4112},
	{111   ,   4097},
	{113   ,   4084},
	{112   ,   4071},
	{114   ,   4056},
	{118   ,   4042},
	{117   ,   4030},
	{121   ,   4019},
	{124   ,   4004},
	{127   ,   3992},
	{128   ,   3982},
	{133   ,   3971},
	{135   ,   3959},
	{135   ,   3948},
	{140   ,   3939},
	{140   ,   3929},
	{142   ,   3918},
	{142   ,   3908},
	{142   ,   3897},
	{136   ,   3885},
	{128   ,   3872},
	{126   ,   3861},
	{120   ,   3850},
	{122   ,   3842},
	{115   ,   3832},
	{112   ,   3823},
	{114   ,   3819},
	{117   ,   3814},
	{113   ,   3806},
	{115   ,   3801},
	{111   ,   3796},
	{111   ,   3791},
	{115   ,   3788},
	{111   ,   3784},
	{113   ,   3780},
	{117   ,   3780},
	{116   ,   3775},
	{118   ,   3774},
	{117   ,   3772},
	{119   ,   3771},
	{119   ,   3769},
	{122   ,   3768},
	{123   ,   3766},
	{123   ,   3764},
	{123   ,   3762},
	{125   ,   3757},
	{125   ,   3752},
	{123   ,   3744},
	{122   ,   3734},
	{121   ,   3722},
	{121   ,   3709},
	{122   ,   3695},
	{122   ,   3686},
	{120   ,   3678},
	{130   ,   3674},
	{140   ,   3668},
	{162   ,   3660},
	{196   ,   3641},
	{225   ,   3573},
	{350   ,   3500},
	{350   ,   3500} 
};    
// T2 25C
R_PROFILE_STRUC r_profile_t2[] =
{
	{72    ,     4193},
	{72    ,     4172},
	{75    ,     4155},
	{74    ,     4138},
	{74    ,     4122},
	{75    ,     4107},
	{76    ,     4092},
	{75    ,	 4078},
	{75    ,	 4063},
	{77    ,	 4050},
	{76    ,	 4037},
	{80    ,	 4024},
	{78    ,	 4011},
	{78    ,	 3998},
	{80    ,	 3987},
	{79    ,	 3976},
	{83    ,	 3965},
	{84    ,	 3954},
	{85    ,	 3945},
	{85    ,	 3934},
	{88    ,	 3926},
	{91    ,	 3916},
	{95    ,	 3907},
	{95    ,	 3898},
	{99    ,	 3889},
	{102   ,	 3881},
	{100   ,	 3869},
	{99    ,	 3858},
	{91    ,	 3846},
	{89    ,	 3834},
	{84    ,	 3823},
	{82    ,	 3814},
	{81    ,	 3807},
	{77    ,	 3800},
	{78    ,	 3794},
	{77    ,	 3789},
	{78    ,	 3785},
	{77    ,	 3780},
	{76    ,	 3776},
	{77    ,	 3772},
	{79    ,	 3770},
	{80    ,	 3768},
	{81    ,	 3764},
	{81    ,	 3763},
	{81    ,	 3759},
	{81    ,	 3758},
	{80    ,	 3756},
	{82    ,	 3753},
	{82    ,	 3752},
	{79    ,	 3747},
	{82    ,	 3742},
	{77    ,	 3733},
	{79    ,	 3726},
	{79    ,	 3720},
	{82    ,	 3710},
	{79    ,	 3696},
	{81    ,	 3683},
	{80    ,	 3669},
	{78    ,	 3664},
	{81    ,	 3660},
	{83    ,	 3655},
	{88    ,	 3649},
	{94    ,	 3634},
	{91    ,	 3578},
	{98    ,	 3512},
	{116   ,     3500}
};    
// T3 50C
R_PROFILE_STRUC r_profile_t3[] =
{
	{73    ,   4191},
	{73    ,   4171},
	{77    ,   4156},
	{73    ,   4138},
	{75    ,   4122},
	{74    ,   4107},
	{73    ,   4091},
	{76    ,   4077},
	{72    ,   4063},
	{72    ,   4048},
	{74    ,   4036},
	{78    ,   4023},
	{71    ,   4009},
	{73    ,   3998},
	{73    ,   3986},
	{73    ,   3974},
	{79    ,   3966},
	{78    ,   3954},
	{80    ,   3942},
	{76    ,   3933},
	{79    ,   3922},
	{75    ,   3913},
	{87    ,   3907},
	{87    ,   3898},
	{83    ,   3887},
	{88    ,   3881},
	{89    ,   3871},
	{91    ,   3860},
	{86    ,   3848},
	{82    ,   3834},
	{79    ,   3824},
	{75    ,   3815},
	{74    ,   3807},
	{77    ,   3803},
	{79    ,   3797},
	{73    ,   3792},
	{73    ,   3785},
	{77    ,   3782},
	{77    ,   3777},
	{76    ,   3773},
	{75    ,   3769},
	{77    ,   3767},
	{76    ,   3764},
	{80    ,   3763},
	{80    ,   3759},
	{80    ,   3756},
	{77    ,   3749},
	{77    ,   3742},
	{73    ,   3736},
	{76    ,   3730},
	{82    ,   3727},
	{71    ,   3716},
	{78    ,   3712},
	{75    ,   3703},
	{76    ,   3691},
	{73    ,   3677},
	{80    ,   3665},
	{74    ,   3656},
	{78    ,   3652},
	{81    ,   3648},
	{77    ,   3643},
	{87    ,   3633},
	{83    ,   3594},
	{83    ,   3513},
	{91    ,   3500},
	{91    ,   3500}
};    
// r-table profile for actual temperature. The size should be the same as T1, T2 and T3
R_PROFILE_STRUC r_profile_temperature[] =
{
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0},
	{0	,  0}	
};

// ============================================================
// function prototype
// ============================================================
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUC_P fgauge_get_profile(kal_uint32 temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUC_P fgauge_get_profile_r_table(kal_uint32 temperature);

#endif	//#ifndef _CUST_BATTERY_METER_TABLE_H
