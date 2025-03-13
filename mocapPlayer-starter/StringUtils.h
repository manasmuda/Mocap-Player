#ifndef __STRING_UTILS_H_
#define __STRING_UTILS_H_

#include <cctype>
#include <cmath>

inline bool isInteger(const char* str, int& outValue) {
    while (*str == ' ' || *str == '\t')
        str++;

    bool isNegative = false;
    if (*str == '-') {
        isNegative = true;
        str++;
    }
    else if (*str == '+') {
        str++;
    }

    if (*str == '\0')
        return false;

    long long result = 0;
    while (*str) {
        if (!std::isdigit(*str))
            return false;

        result = result * 10 + (*str - '0');

        if ((!isNegative && result > INT_MAX) || (isNegative && -result < INT_MIN))
            return false;

        str++;
    }

    outValue = isNegative ? -static_cast<int>(result) : static_cast<int>(result);
    return true;
}

#endif // !__STRING_UTILS_H_
