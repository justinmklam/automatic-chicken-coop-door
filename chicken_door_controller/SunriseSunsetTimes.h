
struct SunriseSunsetTimes {
    uint8_t hour;
    uint8_t minute;
};

struct SunriseSunsetTimes SUNRISE_TIMES[12] = {
    { 8, 7   },    // January
    { 7, 43  },    // February
    { 6, 54  },    // March
    { 6, 49  },    // April
    { 5, 50  },    // May
    { 5, 11  },    // June
    { 5, 11  },    // July
    { 5, 44  },    // August
    { 6, 28  },    // September
    { 7, 12  },    // October
    { 8, 0   },    // November
    { 7, 46  }     // December
};

struct SunriseSunsetTimes SUNSET_TIMES[12] = {
    { 16, 24  },    // January
    { 17, 8   },    // February
    { 17, 55  },    // March
    { 19, 44  },    // April
    { 20, 29  },    // May
    { 21, 9   },    // June
    { 21, 21  },    // July
    { 20, 52  },    // August
    { 19, 54  },    // September
    { 18, 50  },    // October
    { 17, 50  },    // November
    { 16, 16  }     // December
};