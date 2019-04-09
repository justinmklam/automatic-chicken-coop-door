// Takes about 2088 bytes

#define SERIAL_LOG_ENABLED true

void loggerBegin() {
    if (SERIAL_LOG_ENABLED) {
        Serial.begin(115200);
    }
}

void logger(const String &s) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(s);
    }
}

void logger(const char str[]) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(str);
    }
}

void logger(char c) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(c);
    }
}

void logger(int n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(n);
    }
}

void logger(unsigned int n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(n);
    }
}

void logger(long n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(n);
    }
}

void logger(unsigned long n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(n);
    }
}

void logger(double n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.print(n);
    }
}

void loggerln(const String &s) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(s);
    }
}

void loggerln(const char str[]) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(str);
    }
}

void loggerln(char c) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(c);
    }
}

void loggerln(int n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(n);
    }
}

void loggerln(unsigned int n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(n);
    }
}

void loggerln(long n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(n);
    }
}

void loggerln(unsigned long n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(n);
    }
}

void loggerln(double n) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println(n);
    }
}

void loggerln(void) {
    if (SERIAL_LOG_ENABLED) {
        Serial.println();
    }
}